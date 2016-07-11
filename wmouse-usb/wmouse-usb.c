/* Name: main.c
 * Project: custom-class, a basic USB example
 * Author: Christian Starkjohann
 * Creation Date: 2008-04-09
 * Tabsize: 4
 * Copyright: (c) 2008 by OBJECTIVE DEVELOPMENT Software GmbH
 * License: GNU GPL v2 (see License.txt), GNU GPL v3 or proprietary (CommercialLicense.txt)
 * This Revision: $Id$
 */

/*
This example should run on most AVRs with only little changes. No special
hardware resources except INT0 are used. You may have to change usbconfig.h for
different I/O pins for USB. Please note that USB D+ must be the INT0 pin, or
at least be connected to INT0 as well.
We assume that an LED is connected to port B bit 0. If you connect it to a
different port or bit, change the macros below:
*/
#include "config.h"

#include <stdint.h>

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>  /* for sei() */
#include <util/delay.h>     /* for _delay_ms() */

#include <avr/pgmspace.h>   /* required by usbdrv.h */
#include <avr/eeprom.h>   /* required by usbdrv.h */
#include "usbdrv/usbdrv.h"
#include "../nrf24l01_regs.h"

/* ------------------------------------------------------------------------- */
/* ----------------------------- USB interface ----------------------------- */
/* ------------------------------------------------------------------------- */
PROGMEM char usbHidReportDescriptor[52] = { /* USB report descriptor, size must match usbconfig.h */
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x02,                    // USAGE (Mouse)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x09, 0x01,                    //   USAGE (Pointer)
    0xA1, 0x00,                    //   COLLECTION (Physical)
    0x05, 0x09,                    //     USAGE_PAGE (Button)
    0x19, 0x01,                    //     USAGE_MINIMUM
    0x29, 0x03,                    //     USAGE_MAXIMUM
    0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //     LOGICAL_MAXIMUM (1)
    0x95, 0x03,                    //     REPORT_COUNT (3)
    0x75, 0x01,                    //     REPORT_SIZE (1)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0x95, 0x01,                    //     REPORT_COUNT (1)
    0x75, 0x05,                    //     REPORT_SIZE (5)
    0x81, 0x03,                    //     INPUT (Const,Var,Abs)
    0x05, 0x01,                    //     USAGE_PAGE (Generic Desktop)
    0x09, 0x30,                    //     USAGE (X)
    0x09, 0x31,                    //     USAGE (Y)
    0x09, 0x38,                    //     USAGE (Wheel)
    0x15, 0x81,                    //     LOGICAL_MINIMUM (-127)
    0x25, 0x7F,                    //     LOGICAL_MAXIMUM (127)
    0x75, 0x08,                    //     REPORT_SIZE (8)
    0x95, 0x03,                    //     REPORT_COUNT (3)
    0x81, 0x06,                    //     INPUT (Data,Var,Rel)
    0xC0,                          //   END_COLLECTION
    0xC0,                          // END COLLECTION
};
/* This is the same report descriptor as seen in a Logitech mouse. The data
 * described by this descriptor consists of 4 bytes:
 *      .  .  .  .  . B2 B1 B0 .... one byte with mouse button states
 *     X7 X6 X5 X4 X3 X2 X1 X0 .... 8 bit signed relative coordinate x
 *     Y7 Y6 Y5 Y4 Y3 Y2 Y1 Y0 .... 8 bit signed relative coordinate y
 *     W7 W6 W5 W4 W3 W2 W1 W0 .... 8 bit signed relative coordinate wheel
 */
typedef struct{
    uchar   buttonMask;
    char    dx;
    char    dy;
    char    dWheel;
}report_t;

int16_t dxr;
int16_t dyr;
uint8_t btn_tmp;
uint8_t btn;


static report_t reportBuffer;
static uint8_t idleRate;

#if defined (__AVR_ATmega8__)
#define SPI_SCK  5
#define SPI_MISO 4
#define SPI_MOSI 3
#define SPI_SS   2
#endif

#define LED0_ON() DDRD|=(1<<4)
#define LED0_OFF() DDRD&=~(1<<4)

#define N_CS(x) if(x)PORTC|=(1<<1);else PORTC&=~(1<<1)
#define N_CE(x) if(x)PORTC|=(1<<0);else PORTC&=~(1<<0)

#define GET_nIRQ() (PINB & (1<<1))
#define SET_SDN(x) if(x)PORTD|=(1<<5);else PORTD&=~(1<<5)

#define SI_CS(x) if(x)PORTB|=(1<<PB2);else PORTB&=~(1<<PB2)

#define SENS 8

void nset_reg(uint8_t r, uint8_t d,uint8_t ce)
{
	SI_CS(1);
	N_CS(0);
	SPDR=0x20|(r&0x1f);
	while((SPSR&(1<<SPIF))==0);
	SPDR=d;
	while((SPSR&(1<<SPIF))==0);
	N_CS(1);
	N_CE(ce);
}

uint8_t nget_reg(uint8_t r, uint8_t ce)
{
	SI_CS(1);
	N_CS(0);
	SPDR=(r&0x1f);
	while((SPSR&(1<<SPIF))==0);
	SPDR=0xff;
	while((SPSR&(1<<SPIF))==0);
	N_CS(1);
	N_CE(ce);
	return SPDR;
}

void init_spi()
{
	DDRB=(1<<PB2)|(1<<PB3)|(1<<PB5);
	//PORTB=(1<<PB0)|(1<<PB1)|(1<<PB4);
	SPCR=(0<<SPE)|(0<<DORD)|(1<<MSTR)|(0<<CPOL)|(0<<CPHA)|(1<<SPR0)|(1<<SPR1);
	SPCR=(1<<SPE)|(0<<DORD)|(1<<MSTR)|(0<<CPOL)|(0<<CPHA)|(1<<SPR0)|(1<<SPR1);
	SPSR=(0<<SPI2X);
}


usbMsgLen_t usbFunctionSetup(uchar data[8])
{
	struct usbRequest    *rq = (void *)data;
	if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS)
	{    /* class request type */
		switch(rq->bRequest)
		{
		case USBRQ_HID_GET_REPORT:
			usbMsgPtr = (void *)&reportBuffer;
			return sizeof(reportBuffer);
		case USBRQ_HID_GET_IDLE:
			usbMsgPtr = &idleRate;
			return 1;
		case USBRQ_HID_SET_IDLE:
			idleRate = rq->wValue.bytes[1];
		}
	}else{
		/* no vendor specific requests implemented */
		switch(rq->bRequest)
		{
	
		}
	}
	return 0;   /* default for not implemented requests: return no data back to host */
}

void radioInit()
{
	nset_reg(l01_rx_pw_p0,5,0);
	nset_reg(l01_config,nget_reg(l01_config,0)|L01_PWR_UP|L01_PRIM_RX,1);
}
void radioPoll()
{
	int st=0;
	union{
		uint8_t b[4];
		int16_t w[2];
	} buf;
	if((nget_reg(l01_fifo_status,1)&L01_RX_EMPTY)==1)
		return;
	st=nget_reg(l01_status,1);
	nset_reg(l01_status,0xff,1);
	int pipeno=(st&L01_RX_P_NO_MASK)>>L01_RX_P_NO_OFS;
	if(pipeno==7)
	{
		return;
	}
	N_CS(0);
	SPDR=l01_r_rx_payload;
	while((SPSR&(1<<SPIF))==0);
	uint8_t k;
	for(k=0;k<4;k++)
	{
		SPDR=0xff;
		while((SPSR&(1<<SPIF))==0);
		buf.b[k]=SPDR;
	}
	SPDR=0xff;
	while((SPSR&(1<<SPIF))==0);
	btn_tmp=SPDR;
	N_CS(1);
	dxr+=buf.w[0]*SENS;
	dyr-=buf.w[1]*SENS;
	
}


void main(void)
{
	uint8_t i;
	DDRD|=(1<<4)|(1<<5);
	PORTC=(1<<1)|(1<<2);
	DDRC=(1<<0)|(1<<1);
    usbDeviceDisconnect();  /* enforce re-enumeration, do this while interrupts are disabled! */
    usbInit();
    i = 0;
    while(--i){             /* fake USB disconnect for > 250 ms */
        _delay_ms(1);
    }
	init_spi();
	radioInit();
	set_sleep_mode(SLEEP_MODE_IDLE);
	usbDeviceConnect();
	sei();
	DDRD&=~(1<<4);
	reportBuffer.buttonMask=0;
	reportBuffer.dx=0;
	reportBuffer.dy=0;
	reportBuffer.dWheel=0;
    for(;;)
    {                /* main event loop */
		radioPoll();
		LED0_ON();
		usbPoll();
		if(usbInterruptIsReady())
		{
			/* called after every poll of the interrupt endpoint */
			uint8_t found=((reportBuffer.dx!=0)||(reportBuffer.dy!=0));
			reportBuffer.dx=reportBuffer.dy=0;
			int8_t dx=dxr/256;
			int8_t dy=dyr/256;
			if(dx>127)
			{
				reportBuffer.dx=127;
				dxr-=127*256;
				found=1;
			}else if(dx<-127)
			{
				reportBuffer.dx=-127;
				dxr+=127*256;
				found=1;
			}else{
				reportBuffer.dx=dx;
				dxr-=dx*256;
				found|=(reportBuffer.dx!=0);
			}
			
			if(dy>127)
			{
				reportBuffer.dy=127;
				dyr-=127*256;
				found=1;
			}else if(dy<-127)
			{
				reportBuffer.dy=-127;
				dyr+=127*256;
				found=1;
			}else{
				reportBuffer.dy=dy;
				dyr-=dy*256;
				found|=(reportBuffer.dy!=0);
			}
			if(btn!=btn_tmp)
			{
				btn=btn_tmp;
				reportBuffer.buttonMask=(btn?0:1);
				found=1;
			}
			if(found)
			{
				
				usbSetInterrupt((void *)&reportBuffer, sizeof(reportBuffer));
			}
		}
		LED0_OFF();
    };
}

/* ------------------------------------------------------------------------- */
