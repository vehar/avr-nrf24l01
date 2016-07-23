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
#define i2c_delay() asm volatile ("nop; ")
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

#define MPU_I2C_PREFIX ac
#include "../mpu6050.h"
#undef MPU_I2C_PREFIX
#define MPU 104

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

#define BTN_GET() (PINC&(1<<5))


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

void usart_pruint(uint32_t in,uint8_t min)
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
		buf[bp++]=' '-'0';
	while(bp)
	{
		bp--;
		usart_prchar(buf[bp]+'0');
	}
}

void usart_print(int32_t iin,uint8_t min)
{
	uint8_t buf[11];
	uint8_t bp=0;
	uint8_t neg=(iin<0);
	uint32_t in=neg?-iin:iin;
	
	while(in)
	{
		buf[bp++]=in%10;
		in/=10;
	}
	if(bp==0)
		buf[bp++]=0;
	if(neg)
		buf[bp++]='-'-'0';
	while(bp<min)
		buf[bp++]=' '-'0';
	while(bp)
	{
		bp--;
		usart_prchar(buf[bp]+'0');
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

struct mpu_res_i
{
	uint8_t axh,axl,ayh,ayl,azh,azl;
	uint8_t temph,templ;
	uint8_t gxh,gxl,gyh,gyl,gzh,gzl;
};

struct mpu_res_o
{
	int16_t ax,ay,az;
	int16_t temp;
	int16_t gx,gy,gz;
};

union mpu_res
{
	struct mpu_res_i i;
	struct mpu_res_o o;
	uint8_t r[sizeof(struct mpu_res_i)];
};

#define SW(a) do{ tmp=a##l;a##l=a##h;a##h=tmp;}while(0)
void mpu_byteswap(union mpu_res * b)
{
	uint8_t tmp;
	SW(b->i.ax);
	SW(b->i.ay);
	SW(b->i.az);
	SW(b->i.temp);
	SW(b->i.gx);
	SW(b->i.gy);
	SW(b->i.gz);
};
#undef SW

union buf_t{
		uint8_t c[4];
		int16_t i[2];
	};

uint8_t diff(uint16_t a, uint16_t b)
{
	return (a>b)?(a-b):(b-a);
}
#define EBUFSZ 32

int16_t bufx[EBUFSZ];
int16_t bufy[EBUFSZ];
int16_t bufz[EBUFSZ];
uint8_t bufp;
int16_t sumx,sumy,sumz;

#define GDIV 4
#define ADIV 4

void main(void)
{
	uint8_t btn=0xff;
	uint8_t stat=0;
	uint8_t nrr=0,otx;
	uint8_t k;
	union buf_t buf,tb;
	bufp=sumx=sumz=sumy=0;
	N_CE(0);
	N_CS(1);
	init_spi();
	init_usart(9600);
	//sei();
	N_WR(l01_config,L01_MASK_RX_DR|L01_MASK_TX_DS|L01_MASK_MAX_RT|L01_EN_CRC|L01_PWR_UP);
//	N_WR(l01_rf_setup,L01_CONT_WAVE);
	N_WR(l01_rf_setup,L01_RF_DR_HIGH|L01_RF_PWR_0);
	N_WR(l01_rx_pw_p0,6);
	N_WR(l01_rf_ch,2);
	N_WR(l01_setup_retr,0x2f);
	ac_i2cINIT();
	_delay_ms(3000);
/*	for(k=0;k<128;k++)
	{
		ac_i2cSTART();
		if(ac_i2cTX(k<<1))
			usart_pruint(k,0);
		ac_i2cSTOP();
	}*/
	ac_i2cSTART();
	ac_i2cTX((MPU<<1)|1);
	for(k=0;k<0x76;k++)
	{
		usart_prshex(k);usart_prchar('=');
		usart_prshex(ac_i2cRX(1));
		usart_prchar(13);usart_prchar(10);
	}
	ac_i2cSTOP();
	usart_prchar(13);usart_prchar(10);
	
	mpu_write(MPU,mpu_pwr_mgmt_1,MPU_DEVICE_RESET);
	while(mpu_read(MPU,mpu_pwr_mgmt_1)&MPU_DEVICE_RESET);
	mpu_write(MPU,mpu_pwr_mgmt_1,MPU_CLK_GY);
	mpu_write(MPU,mpu_gyro_config,MPU_FS_2K);
	usart_prchar('R');
	usart_prchar(13);usart_prchar(10);
	
	union mpu_res r;
	int32_t tofsx=0;
	int32_t tofsy=0;
	int32_t tofsz=0;
	for(k=0;k<EBUFSZ;k++)
	{
		mpu_read_m(MPU,mpu_accel_xout_h,r.r,sizeof(r));
		mpu_byteswap(&r);
		usart_prchar('a');usart_prchar('x');usart_prchar('=');usart_print(r.o.ax,5);usart_prchar(' ');
		usart_prchar('a');usart_prchar('y');usart_prchar('=');usart_print(r.o.ay,5);usart_prchar(' ');
		usart_prchar('a');usart_prchar('z');usart_prchar('=');usart_print(r.o.az,5);usart_prchar(' ');
	
		usart_prchar('a');usart_prchar('t');usart_prchar('=');usart_print(r.o.temp,5);usart_prchar(' ');
	
		usart_prchar('g');usart_prchar('x');usart_prchar('=');usart_print(r.o.gx/GDIV,5);usart_prchar(' ');
		usart_prchar('g');usart_prchar('y');usart_prchar('=');usart_print(r.o.gy/GDIV,5);usart_prchar(' ');
		usart_prchar('g');usart_prchar('z');usart_prchar('=');usart_print(r.o.gz/GDIV,5);usart_prchar(' ');
		usart_prchar(13);
		tofsx+=r.o.gx;
		tofsy+=r.o.gy;
		tofsz+=r.o.gz;
	};
	usart_prchar(13);usart_prchar(10);
	usart_prchar('o');usart_prchar('x');usart_prchar('=');usart_print(tofsx/EBUFSZ,5);usart_prchar(' ');
	usart_prchar('o');usart_prchar('y');usart_prchar('=');usart_print(tofsy/EBUFSZ,5);usart_prchar(' ');
	usart_prchar('o');usart_prchar('z');usart_prchar('=');usart_print(tofsz/EBUFSZ,5);usart_prchar(' ');
	usart_prchar(13);usart_prchar(10);
	int16_t ofsx=tofsx/EBUFSZ;
	int16_t ofsy=tofsy/EBUFSZ;
	int16_t ofsz=tofsz/EBUFSZ;
	
	
//	usart_prshex(adxl345_read(ADXL,adxl345_data_format));
	usart_prchar(13);usart_prchar(10);
	otx=N_RR(l01_observe_tx);
	usart_prchar('o');usart_prchar('=');usart_prshex(otx);usart_prchar(' ');
	while(1)
	{
//		uint8_t nsam=adxl345_read(ADXL,adxl345_fifo_status)&ADXL345_FIFO_ENTRIES_MASK;
		mpu_read_m(MPU,mpu_accel_xout_h,r.r,sizeof(r));
		mpu_byteswap(&r);
		sumx-=bufx[bufp];bufx[bufp]=r.o.gx/GDIV;sumx+=bufx[bufp];
		sumy-=bufy[bufp];bufy[bufp]=r.o.gy/GDIV;sumy+=bufy[bufp];
		sumz-=bufz[bufp];bufz[bufp]=r.o.gz/GDIV;sumz+=bufz[bufp];
		tb.i[0]=(r.o.gz-ofsz)/GDIV;
		tb.i[1]=(r.o.gy-ofsy)/GDIV;
/*		usart_prchar('g');usart_prchar('z');usart_prchar('=');usart_print(tb.i[0],5);usart_prchar(' ');
		usart_prchar('g');usart_prchar('y');usart_prchar('=');usart_print(tb.i[1],5);usart_prchar(' ');*/
		bufp++;
		if(bufp>=EBUFSZ)
			bufp=0;
/*		tb.i[0]=sumx-ofsx;
		tb.i[1]=sumy-ofsy;*/
/*		usart_prchar('o');usart_prchar('=');usart_prshex(otx);usart_prchar(' ');
		usart_prchar('S');usart_prchar('=');usart_print(nsam,5);usart_prchar(' ');usart_prchar(' ');usart_prchar(' ');
		usart_prchar('x');usart_prchar('=');usart_print(tb.i[0],5);usart_prchar(' ');
		usart_prchar('y');usart_prchar('=');usart_print(tb.i[1],5);usart_prchar(' ');
		usart_prchar('z');usart_prchar('=');usart_print(sumz>>5,5);usart_prchar(' ');*/
		
		nrr=255;
		otx=N_RR(l01_observe_tx);
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
			usart_prchar(10);
		}
		#define MOVETH 2
		if((tb.i[0]>MOVETH)||(tb.i[0]<-MOVETH)||(tb.i[1]>MOVETH)||(tb.i[1]<-MOVETH))
//		if(1)
		{
			N_CS(0);
			N_SPI(l01_w_tx_payload);
			N_SPI(0x00);
			uint8_t k;
			for(k=0;k<sizeof(buf);k++)
				N_SPI(tb.c[k]);
			N_SPI(btn);
			N_CS(1);
			N_CE(1);
// 			_delay_ms(10);
			usart_prchar(13);
		}
/*		N_CE(1);
		_delay_ms(500);
		N_CE(0);
		_delay_ms(500);*/
//			usart_prchar(0x0a);
//			usart_prchar(0x0d);
		//_delay_ms(500);
	}
}
