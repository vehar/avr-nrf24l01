#include <stdint.h>

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <string.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include "../nrf24l01_regs.h"
#include "genubrr.h"

#define i2cpfix ac
#define i2c_delay() asm volatile ("nop;\n\tnop;\n\tnop; ")
//#define i2c_delay() _delay_us(2)

#define i2c_PORT C
#define i2c_SDA_BIT 4
#define i2c_SCL_BIT 5
#include "softI2C.h"
#undef i2cpfix
#undef i2c_delay
#undef i2c_PORT
#undef i2c_SDA_BIT
#undef i2c_SCL_BIT

#define SPI_PORT D
#define SPI_SCK_BIT 4
#define SPI_MOSI_BIT 5
#define SPI_MISO_BIT 6
#define SPI_SETUP 0
#define SPI_IDLE 0
#define spi_delay() asm("nop;")
#define spipfix N
#include "softspi.h"

#define N_CS(x) if(x) PORTD|=(1<<7);else PORTD&=~(1<<7)
#define N_CE(x) if(x) PORTB|=(1<<0);else PORTB&=~(1<<0)

#define BTN_GET() ((PINC&(1<<5))!=0)


#define LED0_ON() DDRD|=(1<<4)
#define LED0_OFF() DDRD&=~(1<<4)

#define LED1_TOGGLE() DDRC^=(1<<3)
#define LED1_ON() DDRC|=(1<<3)
#define LED1_OFF() DDRC&=~(1<<3)



void init_spi()
{
	PORTD=(1<<6)|(1<<2)|(1<<3)|(1<<7);
	DDRD=(1<<1)|(1<<4)|(1<<5)|(1<<7);
	PORTB=0;
	DDRB=(1<<0);
}

void N_WR(uint8_t r,uint8_t v)
{
	N_CS(0);
	N_SPI(l01_w_register|r);
	N_SPI(v);
	N_CS(1);
}

uint8_t N_RR(uint8_t r)
{
	N_CS(0);
	N_SPI(l01_r_register|r);
	uint8_t ret=N_SPI(0xff);
	N_CS(1);
	return ret;
}


uint16_t read_adc(uint8_t x)
{
	ADMUX=(1<<REFS0)|x;
	ADCSRA=(1<<ADEN)|0x04;
	ADCSRA|=(1<<ADSC);
	while(ADCSRA&(1<<ADSC));
	ADCSRA|=(1<<ADSC);
	while(ADCSRA&(1<<ADSC));
	uint16_t ret=ADC;
//	ADMUX=0;
//	ADCSRA=0;
	return ret;
}

#define  init_usart(br) do{\
	UBRRL=GENUBRR(br)&0xff;\
	UBRRH=(GENUBRR(br)>>8)&0xff;\
	UCSRA=GENU2X(br);\
	UCSRC=(1<<URSEL)|(0<<UPM0)|(0<<UPM1)|(0<<USBS)|(1<<UCSZ1)|(1<<UCSZ0);\
	UCSRB=(1<<RXEN)|(1<<TXEN)|(0<<UCSZ2);\
	}while(0)

uint8_t usart_prchar(uint8_t c)
{
	if(!c)
		return 1;
	while(!(UCSRA&(1<<UDRE)));
	UDR=c;
	return 0;
}

void usart_prstr(const uint8_t * S)
{
	uint8_t * p=(uint8_t *)S;
	while(usart_prchar(pgm_read_byte(p++))==0)
		;
}

uint8_t usart_prxchar(uint8_t x)
{
	if(x>=10)
		return usart_prchar(x-10+'A');
	else
		return usart_prchar(x+'0');
}

void usart_print(uint32_t in,uint8_t min)
{
	uint8_t buf[10];
	uint8_t bp=0;
	while(in)
	{
		buf[bp++]=in%10;
		in/=10;
	}
	if(bp==0)
		buf[bp++]=0;
	while(bp<min)
		buf[bp++]=' ';
	while(bp)
	{
		bp--;
		usart_prchar(buf[bp]);
	}
}

void usart_prlhex(uint32_t in,uint8_t bp)
{
	while(bp)
	{
		bp--;
		usart_prxchar( ((uint8_t*)&in)[bp]>>4);
		usart_prxchar( ((uint8_t*)&in)[bp]&0x0f);
	}
}
void usart_prihex(uint16_t in,uint8_t bp)
{
	while(bp)
	{
		bp--;
		usart_prxchar( ((uint8_t*)&in)[bp]>>4);
		usart_prxchar( ((uint8_t*)&in)[bp]&0x0f);
	}
}
void usart_prshex(uint8_t in)
{
	usart_prxchar( in>>4);
	usart_prxchar( in&0x0f);
}


union buf_t{
		uint8_t c[4];
		uint16_t i[2];
	};

uint8_t diff(uint16_t a, uint16_t b)
{
	return (a>b)?(a-b):(b-a);
}


void main(void)
{
	union buf_t buf;
	union buf_t buf1;
	init_spi();
	init_usart(9600);
	//sei();
	N_WR(l01_config,L01_MASK_RX_DR|L01_MASK_TX_DS|L01_MASK_MAX_RT|L01_EN_CRC|L01_PWR_UP);
//	N_WR(l01_rf_setup,L01_CONT_WAVE);
	N_WR(l01_rf_setup,L01_RF_DR_HIGH|L01_RF_PWR_18);
	N_WR(l01_rx_pw_p0,4);
	N_WR(l01_setup_retr,0x2f);
	uint8_t nrr,otx;
	buf1.i[0]=read_adc(2);
	buf1.i[1]=read_adc(3);
	uint8_t btn=0;
	while(1)
	{
/*		N_CE(1);
		_delay_ms(500);
		N_CE(0);
		_delay_ms(500);*/
		buf.i[0]=read_adc(2);
		buf.i[1]=read_adc(3);
		uint8_t btn_tmp=BTN_GET();
		if( (diff(buf.i[0],buf1.i[0])>2)||(diff(buf.i[1],buf1.i[1])>2)||(btn!=btn_tmp))
		{
			union buf_t tb;
			tb.i[0]=buf1.i[0]-buf.i[0];
			tb.i[1]=buf.i[1]-buf1.i[1];
			btn=btn_tmp;
/*			buf1.i[0]=buf.i[0];
			buf1.i[1]=buf.i[1];*/
			nrr=255;
			otx=N_RR(l01_observe_tx);
/*			usart_prchar('o');usart_prchar('=');usart_prshex(otx);
			usart_prchar(';');usart_prchar('s');usart_prchar('=');usart_prshex(N_RR(l01_status));
			usart_prchar(13);usart_prchar(10);*/
			uint8_t stat=0;
			do{
				stat=N_RR(l01_status);
				nrr--;
				if(stat&(L01_TX_DS|L01_MAX_RT))
					N_WR(l01_status,0xff);
				if(nrr==0)
					break;
			}while(stat&L01_TX_FULL);
			if(N_RR(l01_fifo_status)&L01_TX_FULL)
			{
				N_CE(0);
				N_CS(0);
				N_SPI(l01_flush_tx);
				N_CS(1);
				usart_prchar('!');
			}
			usart_prihex(buf.i[0],2);
			usart_prchar(' ');
			usart_prihex(buf.i[1],2);
			N_CS(0);
			N_SPI(l01_w_tx_payload);
			uint8_t k;
			N_SPI(0x00);
			for(k=0;k<sizeof(buf);k++)
				N_SPI(tb.c[k]);
			N_SPI(btn);
			N_CS(1);
			N_CE(1);
	/*		uint8_t k;
			for(k=0;k<0x1d;k++)
			{
				usart_prshex(k);
				usart_prchar('=');
				usart_prshex(N_RR(k));
				usart_prchar(0x0a);
				usart_prchar(0x0d);
			}*/
			usart_prchar(0x0a);
			usart_prchar(0x0d);
			_delay_ms(10);
		}
		//_delay_ms(500);
	}
}
