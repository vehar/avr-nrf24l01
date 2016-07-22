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
PROGMEM
#include "hidreportdescriptor.h"


int16_t dxr;
int16_t dyr;
int16_t axr;
int16_t ayr;
uint8_t btn_tmp;
uint8_t btn;
uint8_t foundr;
uint8_t founda;


static report1_t reportBuffer1;
static report2_t reportBuffer2;
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

#define ASENS 3
#define RSENS 16

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
			switch(rq->wValue.bytes[0])
			{
			case 1:
				usbMsgPtr = (void *)&reportBuffer1;
				return sizeof(reportBuffer1);
			case 2:
				usbMsgPtr = (void *)&reportBuffer2;
				return sizeof(reportBuffer2);
			};
			return 0;
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
	nset_reg(l01_rx_pw_p0,6,0);
	nset_reg(l01_config,nget_reg(l01_config,0)|L01_PWR_UP|L01_PRIM_RX,1);
}
void radioPoll()
{
	int st=0;
	union{
		uint8_t b[4];
		int16_t w[2];
	} buf;
	uint8_t dev=0;
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
	SPDR=0xff;
	while((SPSR&(1<<SPIF))==0);
	dev=SPDR;
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
	if(dev==1)
	{
		if(buf.w[0]>=32767/ASENS)
			axr=32767;
		else if(buf.w[0]<=-32767/ASENS)
			axr=-32767;
		else
			axr=buf.w[0]*ASENS;
		if(buf.w[1]>=32767/ASENS)
			ayr=32767;
		else if(buf.w[1]<=-32767/ASENS)
			ayr=-32767;
		else
			ayr=buf.w[1]*ASENS;
		founda=1;
	}else{
		dxr+=buf.w[0]*RSENS;
		dyr+=buf.w[1]*RSENS;
		foundr=1;
	}
	
}

uint16_t modi(int16_t i)
{
	return (i<0)?-i:i;
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
	reportBuffer1.buttonMask=0;
	reportBuffer1.reportId=1;
	reportBuffer1.dx=0;
	reportBuffer1.dy=0;
	reportBuffer1.dWheel=0;
	reportBuffer2.reportId=2;
	reportBuffer2.x=0;
	reportBuffer2.y=0;
//	reportBuffer2.dWheel=0;
   for(;;)
    {                /* main event loop */
		radioPoll();
		usbPoll();
		
		if((modi(dxr)<256)||(modi(dyr)<256))
			foundr=0;
		reportBuffer1.dx=reportBuffer1.dy=0;
		int8_t dx=dxr/256;
		int8_t dy=dyr/256;
		if(dx>127)
		{
			reportBuffer1.dx=127;
			dxr-=127*256;
			foundr=1;
		}else if(dx<-127)
		{
			reportBuffer1.dx=-127;
			dxr+=127*256;
			foundr=1;
		}else if(dx){
			reportBuffer1.dx=dx;
			dxr-=dx*256;
			foundr|=(reportBuffer1.dx!=0);
		}
		
		if(dy>127)
		{
			reportBuffer1.dy=127;
			dyr-=127*256;
			foundr=1;
		}else if(dy<-127)
		{
			reportBuffer1.dy=-127;
			dyr+=127*256;
			foundr=1;
		}else if(dy){
			reportBuffer1.dy=dy;
			dyr-=dy*256;
			foundr|=(reportBuffer1.dy!=0);
		}
		if(btn!=btn_tmp)
		{
			btn=btn_tmp;
			reportBuffer1.buttonMask=(btn?0:1);
			foundr=1;
		}
/*		reportBuffer1.dx=dxr/64;
		reportBuffer1.dy=dyr/64;*/
		
		if(founda)
		{
			reportBuffer2.x=axr;
			reportBuffer2.y=ayr;
		}
		
		LED0_OFF();
		if(usbInterruptIsReady())
		{
			/* called after every poll of the interrupt endpoint */
			if(foundr)
			{
				
				//LED0_ON();
				usbSetInterrupt((void *)&reportBuffer1, sizeof(reportBuffer1));
		    	foundr=0;
			}else if(founda)
			{
				
				LED0_ON();
				usbSetInterrupt((void *)&reportBuffer2, sizeof(reportBuffer2));
		    	founda=0;
			}
		}
    };
}

/* ------------------------------------------------------------------------- */
