#ifndef ADXL_I2C_PREFIX
#define ADXL_I2C_PREFIX ADXL
#endif
#define __concat(a,b) a##b

#define _concat(a,b) __concat(a,b)

uint8_t adxl345_read(uint8_t addr,uint8_t reg)
{
	_concat(ADXL_I2C_PREFIX,_i2cSTART)();
	_concat(ADXL_I2C_PREFIX,_i2cTX)((addr<<1)|0);
	_concat(ADXL_I2C_PREFIX,_i2cTX)(reg);
	_concat(ADXL_I2C_PREFIX,_i2cSTOP)();
	_concat(ADXL_I2C_PREFIX,_i2cSTART)();
	_concat(ADXL_I2C_PREFIX,_i2cTX)((addr<<1)|1);
	uint8_t ret=_concat(ADXL_I2C_PREFIX,_i2cRX)(0);
	_concat(ADXL_I2C_PREFIX,_i2cSTOP)();
	return ret;
}

uint8_t adxl345_read_m(uint8_t addr,uint8_t reg,uint8_t * buf,uint8_t n)
{
	_concat(ADXL_I2C_PREFIX,_i2cSTART)();
	if(!_concat(ADXL_I2C_PREFIX,_i2cTX)((addr<<1)|0))
	{
		_concat(ADXL_I2C_PREFIX,_i2cSTOP)();
		return 1;
	};
	_concat(ADXL_I2C_PREFIX,_i2cTX)(reg);
	_concat(ADXL_I2C_PREFIX,_i2cSTOP)();
	_concat(ADXL_I2C_PREFIX,_i2cSTART)();
	_concat(ADXL_I2C_PREFIX,_i2cTX)((addr<<1)|1);
	uint8_t k;
	for(k=0;k<n;k++)
		buf[k]=_concat(ADXL_I2C_PREFIX,_i2cRX)((k+1)!=n);
	_concat(ADXL_I2C_PREFIX,_i2cSTOP)();
	return 0;
}

void adxl345_write(uint8_t addr,uint8_t reg,uint8_t data)
{
	_concat(ADXL_I2C_PREFIX,_i2cSTART)();
	_concat(ADXL_I2C_PREFIX,_i2cTX)((addr<<1)|0);
	_concat(ADXL_I2C_PREFIX,_i2cTX)(reg);
	_concat(ADXL_I2C_PREFIX,_i2cTX)(data);
	_concat(ADXL_I2C_PREFIX,_i2cSTOP)();
}

uint8_t adxl345_write_m(uint8_t addr,uint8_t reg,uint8_t * buf,uint8_t n)
{
	_concat(ADXL_I2C_PREFIX,_i2cSTART)();
	if(!_concat(ADXL_I2C_PREFIX,_i2cTX)((addr<<1)|0))
	{
		_concat(ADXL_I2C_PREFIX,_i2cSTOP)();
		return 1;
	};
	_concat(ADXL_I2C_PREFIX,_i2cTX)(reg);
	uint8_t k;
	for(k=0;k<n;k++)
		_concat(ADXL_I2C_PREFIX,_i2cTX)(buf[k]);
	_concat(ADXL_I2C_PREFIX,_i2cSTOP)();
	return 0;
}

#define adxl345_devid          0x00
#define ADXL345_DEVID          0xe5

#define adxl345_thresh_tap     0x1d
#define adxl345_ofsx           0x1e
#define adxl345_ofsy           0x1f
#define adxl345_ofsz           0x20
#define adxl345_dur            0x21
#define adxl345_latent         0x22
#define adxl345_window         0x23
#define adxl345_thresh_act     0x24
#define adxl345_thresh_inact   0x25
#define adxl345_time_inact     0x26

#define adxl345_act_inact_ctl  0x27
#define ADXL345_ACT_ACDC       0x80
#define ADXL345_ACT_X_EN       0x40
#define ADXL345_ACT_Y_EN       0x20
#define ADXL345_ACT_Z_EN       0x10
#define ADXL345_INACT_ACDC     0x08
#define ADXL345_INACT_X_EN     0x04
#define ADXL345_INACT_Y_EN     0x02
#define ADXL345_INACT_Z_EN     0x01

#define adxl345_thresh_ff      0x28
#define adxl345_time_ff        0x29

#define adxl345_tap_axes       0x2a
#define ADXL345_SUPPRESS       0x08
#define ADXL345_TAP_X_EN       0x04
#define ADXL345_TAP_Y_EN       0x02
#define ADXL345_TAP_Z_EN       0x01

#define adxl345_act_tap_status 0x2b
#define ADXL345_ACT_X          0x40
#define ADXL345_ACT_Y          0x20
#define ADXL345_ACT_Z          0x10
#define ADXL345_ASLEEP         0x08
#define ADXL345_TAP_X          0x04
#define ADXL345_TAP_Y          0x02
#define ADXL345_TAP_Z          0x01

#define adxl345_bw_rate        0x2c
#define ADXL345_LOW_POWER      0x10

#define adxl345_power_ctl      0x2d
#define ADXL345_LINK           0x20
#define ADXL345_AUTO_SLEEP     0x10
#define ADXL345_MEASURE        0x08
#define ADXL345_SLEEP          0x04
#define ADXL345_WAKEUP_MASK    0x03
#define ADXL345_WAKEUP8        0x00
#define ADXL345_WAKEUP4        0x01
#define ADXL345_WAKEUP2        0x02
#define ADXL345_WAKEUP1        0x03

#define adxl345_int_enable     0x2e
#define adxl345_int_map        0x2f
#define adxl345_int_source     0x30

#define ADXL345_DATA_READY     0x80
#define ADXL345_SINGLE_TAP     0x40
#define ADXL345_DOUBLE_TAP     0x20
#define ADXL345_ACTIVITY       0x10
#define ADXL345_INACTIVITY     0x08
#define ADXL345_FREE_FALL      0x04
#define ADXL345_WATERMARK      0x02
#define ADXL345_OVERRUN        0x01


#define adxl345_data_format    0x31
#define ADXL345_SELF_TEST      0x80
#define ADXL345_SPI            0x40
#define ADXL345_INT_INVERT     0x20
#define ADXL345_FULL_RES       0x08
#define ADXL345_JUSTIFY        0x04
#define ADXL345_RANGE_MASK     0x03
#define ADXL345_RANGE_2G       0x00
#define ADXL345_RANGE_4G       0x01
#define ADXL345_RANGE_8G       0x02
#define ADXL345_RANGE_16G      0x03

#define adxl345_datx0          0x32
#define adxl345_datx1          0x33
#define adxl345_daty0          0x34
#define adxl345_daty1          0x35
#define adxl345_datz0          0x36
#define adxl345_datz1          0x37

#define adxl345_fifo_ctl       0x38
#define ADXL345_FIFO_MODE_MASK 0xb0
#define ADXL345_FIFO_BYPASS    0xb0
#define ADXL345_FIFO_FIFO      0x40
#define ADXL345_FIFO_STREAM    0x80
#define ADXL345_FIFO_TRIGGER   0xb0
#define ADXL345_TRIGGER        0x20
#define ADXL345_SAMPLES_MASK   0x0f

#define adxl345_fifo_status    0x39
#define ADXL345_FIFO_TRIG      0x80
#define ADXL345_FIFO_ENTRIES_MASK 0x3f








#undef _concat
#undef __concat