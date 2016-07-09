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

#define LED0_ON() DDRD|=(1<<4)
#define LED0_OFF() DDRD&=~(1<<4)

#define LED1_TOGGLE() DDRC^=(1<<3)
#define LED1_ON() DDRC|=(1<<3)
#define LED1_OFF() DDRC&=~(1<<3)



void init_spi()
{
//	DDRB=(1<<PB2)|(1<<PB3)|(1<<PB5);
//	PORTB=(1<<PB4);
	//PORTB=(1<<PB0)|(1<<PB1)|(1<<PB4);
//	SPCR=(0<<SPE)|(0<<DORD)|(1<<MSTR)|(0<<CPOL)|(0<<CPHA)|(0<<SPR0)|(0<<SPR1);
//	SPCR=(1<<SPE)|(0<<DORD)|(1<<MSTR)|(0<<CPOL)|(0<<CPHA)|(0<<SPR0)|(0<<SPR1);
//	SPSR=(0<<SPI2X);
}

uint16_t read_adc(uint8_t x)
{
	ADMUX=(1<<REFS0)|x;
	ADCSRA=(1<<ADEN)|0x07;
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

void usart_prhex(uint32_t in,uint8_t bp)
{
	while(bp)
	{
		bp--;
		usart_prxchar( ((uint8_t*)&in)[bp]>>4);
		usart_prxchar( ((uint8_t*)&in)[bp]&0x0f);
	}
}




void main(void)
{
	init_spi();
	init_usart(9600);
	//sei();
	while(1)
	{
		usart_prhex(read_adc(2),4);
		usart_prchar(' ');
		usart_prhex(read_adc(3),4);
		//usart_prchar(0x0a);
		usart_prchar(0x0d);
		//_delay_ms(500);
	}
}
