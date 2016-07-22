/*

#define spipfix spi0
#define spi_delay() _delay_us(400)

#define SPI_SCK_PORT A
#define SPI_SCK_BIT 0
#define SPI_MOSI_PORT D
#define SPI_MOSI_BIT 1
#define SPI_MISO_PORT B
#define SPI_MISO_BIT 2


*/

#ifndef SPI_SETUP
	#define SPI_SETUP 0
#endif

#ifndef SPI_IDLE
	#define SPI_IDLE 0
#endif


#ifdef SPI_PORT
	#define SPI_SCK_PORT SPI_PORT
	#define SPI_MOSI_PORT SPI_PORT
	#define SPI_MISO_PORT SPI_PORT
#endif


#define __concat(a,b) a##b

#define _concat(a,b) __concat(a,b)

void _concat(spipfix,_SPI_INIT) ()
{
#ifdef 	SPI_PORT
	_concat(DDR,SPI_PORT)&=(~(1<<SPI_MISO_BIT));
	_concat(DDR,SPI_PORT)|=(1<<SPI_SCK_BIT)|(1<<SPI_MOSI_BIT);
	#if SPI_IDLE == 1
	_concat(PORT,SPI_PORT)|=(1<<SPI_SCK_BIT)|(1<<SPI_MOSI_BIT)|(1<<SPI_MISO_BIT);
	#else
	_concat(PORT,SPI_PORT)|=(1<<SPI_MOSI_BIT)|(1<<SPI_MISO_BIT);
	_concat(PORT,SPI_PORT)&=~(1<<SPI_SCK_BIT);
	#endif
#else
	_concat(DDR,SPI_SCK_PORT) |=(1<<SPI_SCK_BIT);
	#if SPI_IDLE == 1
	_concat(PORT,SPI_SCK_PORT)|=(1<<SPI_SCK_BIT);
	#else
	_concat(PORT,SPI_SCK_PORT)&=~(1<<SPI_SCK_BIT);
	#endif
	_concat(DDR,SPI_MOSI_PORT) |=(1<<SPI_MOSI_BIT);
	_concat(PORT,SPI_MOSI_PORT)|=(1<<SPI_MOSI_BIT);
	_concat(DDR,SPI_MISO_PORT) &=~(1<<SPI_MISO_BIT);
	_concat(PORT,SPI_MISO_PORT)|=(1<<SPI_MISO_BIT);
#endif
}

uint8_t _concat(spipfix,_SPI) (uint8_t i)
{
	uint8_t m=0x80;
	uint8_t ret=0;
	while(m)
	{
	#if SPI_SETUP == 0
		#if SPI_IDLE == 0
		if(i&m)
			_concat(PORT,SPI_MOSI_PORT)|=(1<<SPI_MOSI_BIT);
		else
			_concat(PORT,SPI_MOSI_PORT)&=~(1<<SPI_MOSI_BIT);
		spi_delay();
		_concat(PORT,SPI_SCK_PORT)|=(1<<SPI_SCK_BIT);
		spi_delay();
		if(_concat(PIN,SPI_MISO_PORT)&(1<<SPI_MISO_BIT))
			ret|=m;
		_concat(PORT,SPI_SCK_PORT)&=~(1<<SPI_SCK_BIT);
		#else
		if(_concat(PIN,SPI_MISO_PORT)&(1<<SPI_MISO_BIT))
			ret|=m;
		_concat(PORT,SPI_SCK_PORT)&=~(1<<SPI_SCK_BIT);
		spi_delay();
		if(i&m)
			_concat(PORT,SPI_MOSI_PORT)|=(1<<SPI_MOSI_BIT);
		else
			_concat(PORT,SPI_MOSI_PORT)&=~(1<<SPI_MOSI_BIT);
		spi_delay();
		_concat(PORT,SPI_SCK_PORT)|=(1<<SPI_SCK_BIT);
		#endif
	#else
		#if SPI_IDLE == 1
		if(i&m)
			_concat(PORT,SPI_MOSI_PORT)|=(1<<SPI_MOSI_BIT);
		else
			_concat(PORT,SPI_MOSI_PORT)&=~(1<<SPI_MOSI_BIT);
		spi_delay();
		_concat(PORT,SPI_SCK_PORT)&=~(1<<SPI_SCK_BIT);
		spi_delay();
		if(_concat(PIN,SPI_MISO_PORT)&(1<<SPI_MISO_BIT))
			ret|=m;
		_concat(PORT,SPI_SCK_PORT)|=(1<<SPI_SCK_BIT);
		#else
		if(_concat(PIN,SPI_MISO_PORT)&(1<<SPI_MISO_BIT))
			ret|=m;
		_concat(PORT,SPI_SCK_PORT)|=(1<<SPI_SCK_BIT);
		spi_delay();
		if(i&m)
			_concat(PORT,SPI_MOSI_PORT)|=(1<<SPI_MOSI_BIT);
		else
			_concat(PORT,SPI_MOSI_PORT)&=~(1<<SPI_MOSI_BIT);
		spi_delay();
		_concat(PORT,SPI_SCK_PORT)&=~(1<<SPI_SCK_BIT);
		#endif
	#endif
		m>>=1;
	};
	return ret;
}

#ifdef SPI_PORT
#undef SPI_SCK_PORT
#undef SPI_MOSI_PORT
#undef SPI_MISO_PORT
#endif


#undef _concat
