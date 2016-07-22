#ifndef MPU_I2C_PREFIX
#define MPU_I2C_PREFIX MPU
#endif
#define __concat(a,b) a##b

#define _concat(a,b) __concat(a,b)

uint8_t mpu_read(uint8_t addr,uint8_t reg)
{
	_concat(MPU_I2C_PREFIX,_i2cSTART)();
	_concat(MPU_I2C_PREFIX,_i2cTX)((addr<<1)|0);
	_concat(MPU_I2C_PREFIX,_i2cTX)(reg);
	_concat(MPU_I2C_PREFIX,_i2cSTOP)();
	_concat(MPU_I2C_PREFIX,_i2cSTART)();
	_concat(MPU_I2C_PREFIX,_i2cTX)((addr<<1)|1);
	uint8_t ret=_concat(MPU_I2C_PREFIX,_i2cRX)(0);
	_concat(MPU_I2C_PREFIX,_i2cSTOP)();
	return ret;
}

uint8_t mpu_read_m(uint8_t addr,uint8_t reg,uint8_t * buf,uint8_t n)
{
	_concat(MPU_I2C_PREFIX,_i2cSTART)();
	if(!_concat(MPU_I2C_PREFIX,_i2cTX)((addr<<1)|0))
	{
		_concat(MPU_I2C_PREFIX,_i2cSTOP)();
		return 1;
	};
	_concat(MPU_I2C_PREFIX,_i2cTX)(reg);
	_concat(MPU_I2C_PREFIX,_i2cSTOP)();
	_concat(MPU_I2C_PREFIX,_i2cSTART)();
	_concat(MPU_I2C_PREFIX,_i2cTX)((addr<<1)|1);
	uint8_t k;
	for(k=0;k<n;k++)
		buf[k]=_concat(MPU_I2C_PREFIX,_i2cRX)((k+1)!=n);
	_concat(MPU_I2C_PREFIX,_i2cSTOP)();
	return 0;
}

void mpu_write(uint8_t addr,uint8_t reg,uint8_t data)
{
	_concat(MPU_I2C_PREFIX,_i2cSTART)();
	_concat(MPU_I2C_PREFIX,_i2cTX)((addr<<1)|0);
	_concat(MPU_I2C_PREFIX,_i2cTX)(reg);
	_concat(MPU_I2C_PREFIX,_i2cTX)(data);
	_concat(MPU_I2C_PREFIX,_i2cSTOP)();
}

uint8_t mpu_write_m(uint8_t addr,uint8_t reg,uint8_t * buf,uint8_t n)
{
	_concat(MPU_I2C_PREFIX,_i2cSTART)();
	if(!_concat(MPU_I2C_PREFIX,_i2cTX)((addr<<1)|0))
	{
		_concat(MPU_I2C_PREFIX,_i2cSTOP)();
		return 1;
	};
	_concat(MPU_I2C_PREFIX,_i2cTX)(reg);
	uint8_t k;
	for(k=0;k<n;k++)
		_concat(MPU_I2C_PREFIX,_i2cTX)(buf[k]);
	_concat(MPU_I2C_PREFIX,_i2cSTOP)();
	return 0;
}

#define mpu_self_test_x    0x0d
#define MPU_XA_TEST_MASK   0xe0
#define MPU_XA_TEST_OFS    0x05
#define MPU_XG_TEST_MASK   0x1f

#define mpu_self_test_y    0x0e
#define MPU_YA_TEST_MASK   0xe0
#define MPU_YA_TEST_OFS    0x05
#define MPU_YG_TEST_MASK   0x1f

#define mpu_self_test_z    0x0f
#define MPU_ZA_TEST_MASK   0xe0
#define MPU_ZA_TEST_OFS    0x05
#define MPU_ZG_TEST_MASK   0x1f

#define mpu_self_test_a    0x10
#define MPU_XA_TEST_MASK10 0x30
#define MPU_YA_TEST_MASK10 0x0c
#define MPU_ZA_TEST_MASK10 0x03
#define MPU_XA_TEST_OFS10  0x04
#define MPU_YA_TEST_OFS10  0x02
#define MPU_ZA_TEST_OFS10  0x05

#define mpu_smplrt_div        0x19
#define mpu_config            0x1a
#define MPU_EXT_SYNC_SET_MASK 0x38
#define MPU_EXT_SYNC_SET_OFS  0x03
#define MPU_DLPF_CFFG         0x07

#define mpu_gyro_config       0x1b
#define MPU_XG_ST             0x80
#define MPU_YG_ST             0x40
#define MPU_ZG_ST             0x20
#define MPU_FS_SEL_MASK       0x18
#define MPU_FS_SEL_OFS        0x03
#define MPU_FS_250            0x00
#define MPU_FS_500            0x08
#define MPU_FS_1K             0x10
#define MPU_FS_2K             0x18

#define mpu_accel_config      0x1c
#define MPU_XA_ST             0x80
#define MPU_YA_ST             0x40
#define MPU_ZA_ST             0x20
#define MPU_AFS_SEL_MASK      0x18
#define MPU_AFS_SEL_OFS       0x03
#define MPU_FS_2G             0x00
#define MPU_FS_4G             0x08
#define MPU_FS_8G             0x10
#define MPU_FS_16G            0x18

#define mpu_fifo_en           0x23
#define MPU_TEMP_FIFO_EN      0x80
#define MPU_XG_FIFO_EN        0x40
#define MPU_YG_FIFO_EN        0x20
#define MPU_ZG_FIFO_EN        0x10
#define MPU_ACCEL_FIFO_EN0    0x08
#define MPU_SLV2_FIFO_EN0     0x04
#define MPU_SLV1_FIFO_EN0     0x02
#define MPU_SLV0_FIFO_EN0     0x01

#define mpu_i2c_mst_ctrl      0x24
#define MPU_MULT_MST_EN       0x80
#define MPU_WAIT_FOR_ES       0x40
#define MPU_SLV3_FIFO_EN      0x20
#define MPU_I2C_MST_P_NSR     0x10
#define MPU_I2C_MST_CLK_MASK  0x0f

#define mpu_i2c_slv0_addr      0x25
#define MPU_I2C_SLV0_RW        0x80
#define MPU_I2C_SLV0_ADDR_MASK 0x7f

#define mpu_i2c_slv0_reg       0x26

#define mpu_i2c_slv0_ctrl      0x27
#define MPU_I2C_SLV0_EN        0x80
#define MPU_I2C_SLV0_BYTE_SW   0x40
#define MPU_I2C_SLV0_REG_DIS   0x20
#define MPU_I2C_SLV0_GRP       0x10
#define MPU_I2C_SLV0_LEN_MASK  0x0F

#define mpu_i2c_slv1_addr      0x28
#define MPU_I2C_SLV1_RW        0x80
#define MPU_I2C_SLV1_ADDR_MASK 0x7f

#define mpu_i2c_slv1_reg       0x29

#define mpu_i2c_slv1_ctrl      0x2a
#define MPU_I2C_SLV1_EN        0x80
#define MPU_I2C_SLV1_BYTE_SW   0x40
#define MPU_I2C_SLV1_REG_DIS   0x20
#define MPU_I2C_SLV1_GRP       0x10
#define MPU_I2C_SLV1_LEN_MASK  0x0F

#define mpu_i2c_slv2_addr      0x2b
#define MPU_I2C_SLV2_RW        0x80
#define MPU_I2C_SLV2_ADDR_MASK 0x7f

#define mpu_i2c_slv2_reg       0x2c

#define mpu_i2c_slv2_ctrl      0x2d
#define MPU_I2C_SLV2_EN        0x80
#define MPU_I2C_SLV2_BYTE_SW   0x40
#define MPU_I2C_SLV2_REG_DIS   0x20
#define MPU_I2C_SLV2_GRP       0x10
#define MPU_I2C_SLV2_LEN_MASK  0x0F

#define mpu_i2c_slv3_addr      0x2e
#define MPU_I2C_SLV3_RW        0x80
#define MPU_I2C_SLV3_ADDR_MASK 0x7f

#define mpu_i2c_slv3_reg       0x2f

#define mpu_i2c_slv3_ctrl      0x30
#define MPU_I2C_SLV3_EN        0x80
#define MPU_I2C_SLV3_BYTE_SW   0x40
#define MPU_I2C_SLV3_REG_DIS   0x20
#define MPU_I2C_SLV3_GRP       0x10
#define MPU_I2C_SLV3_LEN_MASK  0x0F

#define mpu_i2c_slv4_addr      0x31
#define MPU_I2C_SLV4_RW        0x80
#define MPU_I2C_SLV4_ADDR_MASK 0x7f

#define mpu_i2c_slv4_reg       0x32

#define mpu_i2c_slv4_do        0x33

#define mpu_i2c_slv4_ctrl      0x34
#define MPU_I2C_SLV4_EN        0x80
#define MPU_I2C_SLV4_BYTE_SW   0x40
#define MPU_I2C_SLV4_REG_DIS   0x20
#define MPU_I2C_SLV4_GRP       0x10
#define MPU_I2C_SLV4_LEN_MASK  0x0F

#define mpu_i2c_slv4_di        0x35

#define mpu_i2c_mst_status     0x36
#define MPU_PASS_THROUGH       0x80
#define MPU_I2C_SLV4_DONE      0x40
#define MPU_I2C_LOST_ARB       0x20
#define MPU_I2C_SLV4_NACK      0x10
#define MPU_I2C_SLV3_NACK      0x08
#define MPU_I2C_SLV2_NACK      0x04
#define MPU_I2C_SLV1_NACK      0x02
#define MPU_I2C_SLV0_NACK      0x01

#define mpu_int_pin_cfg        0x37
#define MPU_INT_LEVEL          0x80
#define MPU_INT_OPEN           0x40
#define MPU_LATCH_INT_EN       0x20
#define MPU_INT_RD_CLEAR       0x10
#define MPU_FSYNC_INT_LEVEL    0x08
#define MPU_FSYNC_INT_EN       0x04
#define MPU_I2C_BYPASS_EN      0x02

#define mpu_int_enable         0x38

#define mpu_int_status         0x3a
#define MPU_FIFO_OFLOW_INT     0x10
#define MPU_I2C_MST_INT        0x08
#define MPU_DATA_RDY_INT       0x04

#define mpu_accel_xout_h       0x3b

#define mpu_accel_xout_l       0x3c

#define mpu_accel_yout_h       0x3d

#define mpu_accel_yout_l       0x3e

#define mpu_accel_zout_h       0x3f

#define mpu_accel_zout_l       0x40

#define mpu_temp_out_h         0x41

#define mpu_temp_out_l         0x42

#define mpu_gyro_xout_h        0x43

#define mpu_gyro_xout_l        0x44

#define mpu_gyro_yout_h        0x45

#define mpu_gyro_yout_l        0x46

#define mpu_gyro_zout_h        0x47

#define mpu_gyro_zout_l        0x48

#define mpu_ext_sens_data00    0x49

#define mpu_ext_sens_data01    0x4a

#define mpu_ext_sens_data02    0x4b

#define mpu_ext_sens_data03    0x4c

#define mpu_ext_sens_data04    0x4d

#define mpu_ext_sens_data05    0x4e

#define mpu_ext_sens_data06    0x4f

#define mpu_ext_sens_data07    0x50

#define mpu_ext_sens_data08    0x51

#define mpu_ext_sens_data09    0x52

#define mpu_ext_sens_data10    0x53

#define mpu_ext_sens_data11    0x54

#define mpu_ext_sens_data12    0x55

#define mpu_ext_sens_data13    0x56

#define mpu_ext_sens_data14    0x57

#define mpu_ext_sens_data15    0x58

#define mpu_ext_sens_data16    0x59

#define mpu_ext_sens_data17    0x5a

#define mpu_ext_sens_data18    0x5b

#define mpu_ext_sens_data19    0x5c

#define mpu_ext_sens_data20    0x5d

#define mpu_ext_sens_data21    0x5e

#define mpu_ext_sens_data22    0x5f

#define mpu_ext_sens_data23    0x60

#define mpu_i2c_slv0_do        0x63

#define mpu_i2c_slv1_do        0x64

#define mpu_i2c_slv2_do        0x65

#define mpu_i2c_slv3_do        0x66

#define mpu_i2c_mst_delay_ctr  0x67
#define MPU_DELAY_ES_SHADOW    0x80
#define MPU_I2C_SLV4_DLY_EN    0x10
#define MPU_I2C_SLV3_DLY_EN    0x08
#define MPU_I2C_SLV2_DLY_EN    0x04
#define MPU_I2C_SLV1_DLY_EN    0x02
#define MPU_I2C_SLV0_DLY_EN    0x01

#define mpu_signal_path_reset  0x68
#define MPU_GYRO_RESET         0x04
#define MPU_ACCEL_RESET        0x02
#define MPU_TEMP_RESET         0x01

#define mpu_user_ctrl          0x6a
#define MPU_FIFO_EN            0x40
#define MPU_I2C_MST_EN         0x20
#define MPU_I2C_IF_DIS         0x10
#define MPU_FIFO_RESET         0x04
#define MPU_I2C_MST_RESET      0x02
#define MPU_SIG_COND_RESET     0x01

#define mpu_pwr_mgmt_1         0x6b
#define MPU_DEVICE_RESET       0x80
#define MPU_SLEEP              0x40
#define MPU_CYCLE              0x20
#define MPU_TEMP_DIS           0x08
#define MPU_CLKSEL_MASK        0x07
#define MPU_CLK_INT8           0x00
#define MPU_CLK_GX             0x01
#define MPU_CLK_GY             0x02
#define MPU_CLK_GZ             0x03
#define MPU_CLK_EXT32K         0x04
#define MPU_CLK_EXT19M         0x05
#define MPU_CLK_STOP           0x07

#define mpu_pwr_mgmt_2         0x6c
#define MPU_LP_WAKE_CTRL_MASK  0xc0
#define MPU_STBY_XA            0x20
#define MPU_STBY_YA            0x10
#define MPU_STBY_ZA            0x08
#define MPU_STBY_XG            0x04
#define MPU_STBY_YG            0x02
#define MPU_STBY_ZG            0x01

#define mpu_fifo_count_h       0x72

#define mpu_fifo_count_l       0x73

#define mpu_fifo_r_w           0x74

#define mpu_who_am_i           0x75



#undef _concat
#undef __concat