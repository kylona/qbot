#![allow(unused)]

// I2Cdev library collection - MPU9250 I2C device class
// Based on InvenSense MPU-9250 register map document rev. 2.0, 5/19/2011 (RM-MPU-6000A-00)
// 8/24/2011 by Jeff Rowberg <jeff@rowberg.net>
// Ported to Rust by Kyle Storey <kyle@kylona.com>

// NOTE: THIS IS ONLY A PARIAL RELEASE. THIS DEVICE CLASS IS CURRENTLY UNDERGOING ACTIVE
// DEVELOPMENT AND IS STILL MISSING SOME IMPORTANT FEATURES. PLEASE KEEP THIS IN MIND IF
// YOU DECIDE TO USE THIS PARTICULAR CODE FOR ANYTHING.

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

use crate::i2c;
use anyhow::Result;

pub mod mpu9150 {

    //Magnetometer Registers
    const RA_MAG_ADDRESS : u16 = 0x0C;
    const RA_MAG_XOUT_L	 : u8 = 0x03;
    const RA_MAG_XOUT_H	 : u8 = 0x04;
    const RA_MAG_YOUT_L	 : u8 = 0x05;
    const RA_MAG_YOUT_H	 : u8 = 0x06;
    const RA_MAG_ZOUT_L	 : u8 = 0x07;
    const RA_MAG_ZOUT_H	 : u8 = 0x08;

}

const ADDRESS_AD0_LOW    : u16 = 0x68; // address pin low (GND), default for InvenSense evaluation board
const ADDRESS_AD0_HIGH   : u16 = 0x69; // address pin high (VCC)
const DEFAULT_ADDRESS    : u16 = ADDRESS_AD0_LOW;

const RA_XG_OFFS_TC : u8 = 0x00; //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
const RA_YG_OFFS_TC : u8 = 0x01; //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
const RA_ZG_OFFS_TC : u8 = 0x02; //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
const RA_X_FINE_GAIN : u8 = 0x03; //[7:0] X_FINE_GAIN
const RA_Y_FINE_GAIN : u8 = 0x04; //[7:0] Y_FINE_GAIN
const RA_Z_FINE_GAIN : u8 = 0x05; //[7:0] Z_FINE_GAIN
const RA_XA_OFFS_H : u8 = 0x06; //[15:0] XA_OFFS
const RA_XA_OFFS_L_TC : u8 = 0x07;
const RA_YA_OFFS_H : u8 = 0x08; //[15:0] YA_OFFS
const RA_YA_OFFS_L_TC : u8 = 0x09;
const RA_ZA_OFFS_H : u8 = 0x0A; //[15:0] ZA_OFFS
const RA_ZA_OFFS_L_TC : u8 = 0x0B;
const RA_XG_OFFS_USRH : u8 = 0x13; //[15:0] XG_OFFS_USR
const RA_XG_OFFS_USRL : u8 = 0x14;
const RA_YG_OFFS_USRH : u8 = 0x15; //[15:0] YG_OFFS_USR
const RA_YG_OFFS_USRL : u8 = 0x16;
const RA_ZG_OFFS_USRH : u8 = 0x17; //[15:0] ZG_OFFS_USR
const RA_ZG_OFFS_USRL : u8 = 0x18;
const RA_SMPLRT_DIV : u8 = 0x19;
const RA_CONFIG : u8 = 0x1A;
const RA_GYRO_CONFIG : u8 = 0x1B;
const RA_ACCEL_CONFIG : u8 = 0x1C;
const RA_FF_THR : u8 = 0x1D;
const RA_FF_DUR : u8 = 0x1E;
const RA_MOT_THR : u8 = 0x1F;
const RA_MOT_DUR : u8 = 0x20;
const RA_ZRMOT_THR : u8 = 0x21;
const RA_ZRMOT_DUR : u8 = 0x22;
const RA_FIFO_EN : u8 = 0x23;
const RA_I2C_MST_CTRL : u8 = 0x24;
const RA_I2C_SLV0_ADDR : u8 = 0x25;
const RA_I2C_SLV0_REG : u8 = 0x26;
const RA_I2C_SLV0_CTRL : u8 = 0x27;
const RA_I2C_SLV1_ADDR : u8 = 0x28;
const RA_I2C_SLV1_REG : u8 = 0x29;
const RA_I2C_SLV1_CTRL : u8 = 0x2A;
const RA_I2C_SLV2_ADDR : u8 = 0x2B;
const RA_I2C_SLV2_REG : u8 = 0x2C;
const RA_I2C_SLV2_CTRL : u8 = 0x2D;
const RA_I2C_SLV3_ADDR : u8 = 0x2E;
const RA_I2C_SLV3_REG : u8 = 0x2F;
const RA_I2C_SLV3_CTRL : u8 = 0x30;
const RA_I2C_SLV4_ADDR : u8 = 0x31;
const RA_I2C_SLV4_REG : u8 = 0x32;
const RA_I2C_SLV4_DO : u8 = 0x33;
const RA_I2C_SLV4_CTRL : u8 = 0x34;
const RA_I2C_SLV4_DI : u8 = 0x35;
const RA_I2C_MST_STATUS : u8 = 0x36;
const RA_INT_PIN_CFG : u8 = 0x37;
const RA_INT_ENABLE : u8 = 0x38;
const RA_DMP_INT_STATUS : u8 = 0x39;
const RA_INT_STATUS : u8 = 0x3A;
const RA_ACCEL_XOUT_H : u8 = 0x3B;
const RA_ACCEL_XOUT_L : u8 = 0x3C;
const RA_ACCEL_YOUT_H : u8 = 0x3D;
const RA_ACCEL_YOUT_L : u8 = 0x3E;
const RA_ACCEL_ZOUT_H : u8 = 0x3F;
const RA_ACCEL_ZOUT_L : u8 = 0x40;
const RA_TEMP_OUT_H : u8 = 0x41;
const RA_TEMP_OUT_L : u8 = 0x42;
const RA_GYRO_XOUT_H : u8 = 0x43;
const RA_GYRO_XOUT_L : u8 = 0x44;
const RA_GYRO_YOUT_H : u8 = 0x45;
const RA_GYRO_YOUT_L : u8 = 0x46;
const RA_GYRO_ZOUT_H : u8 = 0x47;
const RA_GYRO_ZOUT_L : u8 = 0x48;
const RA_EXT_SENS_DATA_00 : u8 = 0x49;
const RA_EXT_SENS_DATA_01 : u8 = 0x4A;
const RA_EXT_SENS_DATA_02 : u8 = 0x4B;
const RA_EXT_SENS_DATA_03 : u8 = 0x4C;
const RA_EXT_SENS_DATA_04 : u8 = 0x4D;
const RA_EXT_SENS_DATA_05 : u8 = 0x4E;
const RA_EXT_SENS_DATA_06 : u8 = 0x4F;
const RA_EXT_SENS_DATA_07 : u8 = 0x50;
const RA_EXT_SENS_DATA_08 : u8 = 0x51;
const RA_EXT_SENS_DATA_09 : u8 = 0x52;
const RA_EXT_SENS_DATA_10 : u8 = 0x53;
const RA_EXT_SENS_DATA_11 : u8 = 0x54;
const RA_EXT_SENS_DATA_12 : u8 = 0x55;
const RA_EXT_SENS_DATA_13 : u8 = 0x56;
const RA_EXT_SENS_DATA_14 : u8 = 0x57;
const RA_EXT_SENS_DATA_15 : u8 = 0x58;
const RA_EXT_SENS_DATA_16 : u8 = 0x59;
const RA_EXT_SENS_DATA_17 : u8 = 0x5A;
const RA_EXT_SENS_DATA_18 : u8 = 0x5B;
const RA_EXT_SENS_DATA_19 : u8 = 0x5C;
const RA_EXT_SENS_DATA_20 : u8 = 0x5D;
const RA_EXT_SENS_DATA_21 : u8 = 0x5E;
const RA_EXT_SENS_DATA_22 : u8 = 0x5F;
const RA_EXT_SENS_DATA_23 : u8 = 0x60;
const RA_MOT_DETECT_STATUS : u8 = 0x61;
const RA_I2C_SLV0_DO : u8 = 0x63;
const RA_I2C_SLV1_DO : u8 = 0x64;
const RA_I2C_SLV2_DO : u8 = 0x65;
const RA_I2C_SLV3_DO : u8 = 0x66;
const RA_I2C_MST_DELAY_CTRL : u8 = 0x67;
const RA_SIGNAL_PATH_RESET : u8 = 0x68;
const RA_MOT_DETECT_CTRL : u8 = 0x69;
const RA_USER_CTRL : u8 = 0x6A;
const RA_PWR_MGMT_1 : u8 = 0x6B;
const RA_PWR_MGMT_2 : u8 = 0x6C;
const RA_BANK_SEL : u8 = 0x6D;
const RA_MEM_START_ADDR : u8 = 0x6E;
const RA_MEM_R_W : u8 = 0x6F;
const RA_DMP_CFG_1 : u8 = 0x70;
const RA_DMP_CFG_2 : u8 = 0x71;
const RA_FIFO_COUNTH : u8 = 0x72;
const RA_FIFO_COUNTL : u8 = 0x73;
const RA_FIFO_R_W : u8 = 0x74;
const RA_WHO_AM_I : u8 = 0x75;

const TC_PWR_MODE_BIT : u8 = 7;
const TC_OFFSET_BIT : u8 = 6;
const TC_OFFSET_LENGTH : u8 = 6;
const TC_OTP_BNK_VLD_BIT : u8 = 0;

const VDDIO_LEVEL_VLOGIC : u8 = 0;
const VDDIO_LEVEL_VDD : u8 = 1;

const CFG_EXT_SYNC_SET_BIT : u8 = 5;
const CFG_EXT_SYNC_SET_LENGTH : u8 = 3;
const CFG_DLPF_CFG_BIT : u8 = 2;
const CFG_DLPF_CFG_LENGTH : u8 = 3;

const EXT_SYNC_DISABLED : u8 = 0x0;
const EXT_SYNC_TEMP_OUT_L : u8 = 0x1;
const EXT_SYNC_GYRO_XOUT_L : u8 = 0x2;
const EXT_SYNC_GYRO_YOUT_L : u8 = 0x3;
const EXT_SYNC_GYRO_ZOUT_L : u8 = 0x4;
const EXT_SYNC_ACCEL_XOUT_L : u8 = 0x5;
const EXT_SYNC_ACCEL_YOUT_L : u8 = 0x6;
const EXT_SYNC_ACCEL_ZOUT_L : u8 = 0x7;

const DLPF_BW_256 : u8 = 0x00;
const DLPF_BW_188 : u8 = 0x01;
const DLPF_BW_98 : u8 = 0x02;
const DLPF_BW_42 : u8 = 0x03;
const DLPF_BW_20 : u8 = 0x04;
const DLPF_BW_10 : u8 = 0x05;
const DLPF_BW_5 : u8 = 0x06;

const GCONFIG_FS_SEL_BIT : u8 = 4;
const GCONFIG_FS_SEL_LENGTH : u8 = 2;

const GYRO_FS_250 : u8 = 0x00;
const GYRO_FS_500 : u8 = 0x01;
const GYRO_FS_1000 : u8 = 0x02;
const GYRO_FS_2000 : u8 = 0x03;

const ACONFIG_XA_ST_BIT : u8 = 7;
const ACONFIG_YA_ST_BIT : u8 = 6;
const ACONFIG_ZA_ST_BIT : u8 = 5;
const ACONFIG_AFS_SEL_BIT : u8 = 4;
const ACONFIG_AFS_SEL_LENGTH : u8 = 2;
const ACONFIG_ACCEL_HPF_BIT : u8 = 2;
const ACONFIG_ACCEL_HPF_LENGTH : u8 = 3;

const ACCEL_FS_2 : u8 = 0x00;
const ACCEL_FS_4 : u8 = 0x01;
const ACCEL_FS_8 : u8 = 0x02;
const ACCEL_FS_16 : u8 = 0x03;

const DHPF_RESET : u8 = 0x00;
const DHPF_5 : u8 = 0x01;
const DHPF_2P5 : u8 = 0x02;
const DHPF_1P25 : u8 = 0x03;
const DHPF_0P63 : u8 = 0x04;
const DHPF_HOLD : u8 = 0x07;

const TEMP_FIFO_EN_BIT : u8 = 7;
const XG_FIFO_EN_BIT : u8 = 6;
const YG_FIFO_EN_BIT : u8 = 5;
const ZG_FIFO_EN_BIT : u8 = 4;
const ACCEL_FIFO_EN_BIT : u8 = 3;
const SLV2_FIFO_EN_BIT : u8 = 2;
const SLV1_FIFO_EN_BIT : u8 = 1;
const SLV0_FIFO_EN_BIT : u8 = 0;

const MULT_MST_EN_BIT : u8 = 7;
const WAIT_FOR_ES_BIT : u8 = 6;
const SLV_3_FIFO_EN_BIT : u8 = 5;
const I2C_MST_P_NSR_BIT : u8 = 4;
const I2C_MST_CLK_BIT : u8 = 3;
const I2C_MST_CLK_LENGTH : u8 = 4;
const CLOCK_DIV_348 : u8 = 0x0;
const CLOCK_DIV_333 : u8 = 0x1;
const CLOCK_DIV_320 : u8 = 0x2;
const CLOCK_DIV_308 : u8 = 0x3;
const CLOCK_DIV_296 : u8 = 0x4;
const CLOCK_DIV_286 : u8 = 0x5;
const CLOCK_DIV_276 : u8 = 0x6;
const CLOCK_DIV_267 : u8 = 0x7;
const CLOCK_DIV_258 : u8 = 0x8;
const CLOCK_DIV_500 : u8 = 0x9;
const CLOCK_DIV_471 : u8 = 0xA;
const CLOCK_DIV_444 : u8 = 0xB;
const CLOCK_DIV_421 : u8 = 0xC;
const CLOCK_DIV_400 : u8 = 0xD;
const CLOCK_DIV_381 : u8 = 0xE;
const CLOCK_DIV_364 : u8 = 0xF;

const I2C_SLV_RW_BIT : u8 = 7;
const I2C_SLV_ADDR_BIT : u8 = 6;
const I2C_SLV_ADDR_LENGTH : u8 = 7;
const I2C_SLV_EN_BIT : u8 = 7;
const I2C_SLV_BYTE_SW_BIT : u8 = 6;
const I2C_SLV_REG_DIS_BIT : u8 = 5;
const I2C_SLV_GRP_BIT : u8 = 4;
const I2C_SLV_LEN_BIT : u8 = 3;
const I2C_SLV_LEN_LENGTH : u8 = 4;

const I2C_SLV4_RW_BIT : u8 = 7;
const I2C_SLV4_ADDR_BIT : u8 = 6;
const I2C_SLV4_ADDR_LENGTH : u8 = 7;
const I2C_SLV4_EN_BIT : u8 = 7;
const I2C_SLV4_INT_EN_BIT : u8 = 6;
const I2C_SLV4_REG_DIS_BIT : u8 = 5;
const I2C_SLV4_MST_DLY_BIT : u8 = 4;
const I2C_SLV4_MST_DLY_LENGTH : u8 = 5;

const MST_PASS_THROUGH_BIT : u8 = 7;
const MST_I2C_SLV4_DONE_BIT : u8 = 6;
const MST_I2C_LOST_ARB_BIT : u8 = 5;
const MST_I2C_SLV4_NACK_BIT : u8 = 4;
const MST_I2C_SLV3_NACK_BIT : u8 = 3;
const MST_I2C_SLV2_NACK_BIT : u8 = 2;
const MST_I2C_SLV1_NACK_BIT : u8 = 1;
const MST_I2C_SLV0_NACK_BIT : u8 = 0;

const INTCFG_INT_LEVEL_BIT : u8 = 7;
const INTCFG_INT_OPEN_BIT : u8 = 6;
const INTCFG_LATCH_INT_EN_BIT : u8 = 5;
const INTCFG_INT_RD_CLEAR_BIT : u8 = 4;
const INTCFG_FSYNC_INT_LEVEL_BIT : u8 = 3;
const INTCFG_FSYNC_INT_EN_BIT : u8 = 2;
const INTCFG_I2C_BYPASS_EN_BIT : u8 = 1;
const INTCFG_CLKOUT_EN_BIT : u8 = 0;

const INTMODE_ACTIVEHIGH : u8 = 0x00;
const INTMODE_ACTIVELOW : u8 = 0x01;

const INTDRV_PUSHPULL : u8 = 0x00;
const INTDRV_OPENDRAIN : u8 = 0x01;

const INTLATCH_50USPULSE : u8 = 0x00;
const INTLATCH_WAITCLEAR : u8 = 0x01;

const INTCLEAR_STATUSREAD : u8 = 0x00;
const INTCLEAR_ANYREAD : u8 = 0x01;

const INTERRUPT_FF_BIT : u8 = 7;
const INTERRUPT_MOT_BIT : u8 = 6;
const INTERRUPT_ZMOT_BIT : u8 = 5;
const INTERRUPT_FIFO_OFLOW_BIT : u8 = 4;
const INTERRUPT_I2C_MST_INT_BIT : u8 = 3;
const INTERRUPT_PLL_RDY_INT_BIT : u8 = 2;
const INTERRUPT_DMP_INT_BIT : u8 = 1;
const INTERRUPT_DATA_RDY_BIT : u8 = 0;

//; TODO: figure out what these actually do
// UMPL source code is not very obivous
const DMPINT_5_BIT : u8 = 5;
const DMPINT_4_BIT : u8 = 4;
const DMPINT_3_BIT : u8 = 3;
const DMPINT_2_BIT : u8 = 2;
const DMPINT_1_BIT : u8 = 1;
const DMPINT_0_BIT : u8 = 0;

const MOTION_MOT_XNEG_BIT : u8 = 7;
const MOTION_MOT_XPOS_BIT : u8 = 6;
const MOTION_MOT_YNEG_BIT : u8 = 5;
const MOTION_MOT_YPOS_BIT : u8 = 4;
const MOTION_MOT_ZNEG_BIT : u8 = 3;
const MOTION_MOT_ZPOS_BIT : u8 = 2;
const MOTION_MOT_ZRMOT_BIT : u8 = 0;

const DELAYCTRL_DELAY_ES_SHADOW_BIT : u8 = 7;
const DELAYCTRL_I2C_SLV4_DLY_EN_BIT : u8 = 4;
const DELAYCTRL_I2C_SLV3_DLY_EN_BIT : u8 = 3;
const DELAYCTRL_I2C_SLV2_DLY_EN_BIT : u8 = 2;
const DELAYCTRL_I2C_SLV1_DLY_EN_BIT : u8 = 1;
const DELAYCTRL_I2C_SLV0_DLY_EN_BIT : u8 = 0;

const PATHRESET_GYRO_RESET_BIT : u8 = 2;
const PATHRESET_ACCEL_RESET_BIT : u8 = 1;
const PATHRESET_TEMP_RESET_BIT : u8 = 0;

const DETECT_ACCEL_ON_DELAY_BIT : u8 = 5;
const DETECT_ACCEL_ON_DELAY_LENGTH : u8 = 2;
const DETECT_FF_COUNT_BIT : u8 = 3;
const DETECT_FF_COUNT_LENGTH : u8 = 2;
const DETECT_MOT_COUNT_BIT : u8 = 1;
const DETECT_MOT_COUNT_LENGTH : u8 = 2;

const DETECT_DECREMENT_RESET : u8 = 0x0;
const DETECT_DECREMENT_1 : u8 = 0x1;
const DETECT_DECREMENT_2 : u8 = 0x2;
const DETECT_DECREMENT_4 : u8 = 0x3;

const USERCTRL_DMP_EN_BIT : u8 = 7;
const USERCTRL_FIFO_EN_BIT : u8 = 6;
const USERCTRL_I2C_MST_EN_BIT : u8 = 5;
const USERCTRL_I2C_IF_DIS_BIT : u8 = 4;
const USERCTRL_DMP_RESET_BIT : u8 = 3;
const USERCTRL_FIFO_RESET_BIT : u8 = 2;
const USERCTRL_I2C_MST_RESET_BIT : u8 = 1;
const USERCTRL_SIG_COND_RESET_BIT : u8 = 0;

const PWR1_DEVICE_RESET_BIT : u8 = 7;
const PWR1_SLEEP_BIT : u8 = 6;
const PWR1_CYCLE_BIT : u8 = 5;
const PWR1_TEMP_DIS_BIT : u8 = 3;
const PWR1_CLKSEL_BIT : u8 = 2;
const PWR1_CLKSEL_LENGTH : u8 = 3;

const CLOCK_INTERNAL : u8 = 0x00;
const CLOCK_PLL_XGYRO : u8 = 0x01;
const CLOCK_PLL_YGYRO : u8 = 0x02;
const CLOCK_PLL_ZGYRO : u8 = 0x03;
const CLOCK_PLL_EXT32K : u8 = 0x04;
const CLOCK_PLL_EXT19M : u8 = 0x05;
const CLOCK_KEEP_RESET : u8 = 0x07;

const PWR2_LP_WAKE_CTRL_BIT : u8 = 7;
const PWR2_LP_WAKE_CTRL_LENGTH : u8 = 2;
const PWR2_STBY_XA_BIT : u8 = 5;
const PWR2_STBY_YA_BIT : u8 = 4;
const PWR2_STBY_ZA_BIT : u8 = 3;
const PWR2_STBY_XG_BIT : u8 = 2;
const PWR2_STBY_YG_BIT : u8 = 1;
const PWR2_STBY_ZG_BIT : u8 = 0;

const WAKE_FREQ_1P25 : u8 = 0x0;
const WAKE_FREQ_2P5 : u8 = 0x1;
const WAKE_FREQ_5 : u8 = 0x2;
const WAKE_FREQ_10 : u8 = 0x3;

const BANKSEL_PRFTCH_EN_BIT : u8 = 6;
const BANKSEL_CFG_USER_BANK_BIT : u8 = 5;
const BANKSEL_MEM_SEL_BIT : u8 = 4;
const BANKSEL_MEM_SEL_LENGTH : u8 = 5;

const WHO_AM_I_BIT : u8 = 6;
const WHO_AM_I_LENGTH : u8 = 8;

const DMP_MEMORY_BANKS : u8 = 8;
const DMP_MEMORY_BANK_SIZE : u16 = 256;
const DMP_MEMORY_CHUNK_SIZE : u8 = 16;


/** Specific address constructor.
* @param address I2C address
* @see MPU9250_DEFAULT_ADDRESS
* @see MPU9250_ADDRESS_AD0_LOW
* @see MPU9250_ADDRESS_AD0_HIGH
*/
pub struct MPU9250 {
    pub dev_address : u16,
}
impl MPU9250 {
  pub fn new(dev_address : u16) -> Self {
    Self {
        dev_address: dev_address,
    }
  }

  /** Power on and prepare for general usage.
   * This will activate the device and take it out of sleep mode (which must be done
   * after start-up). This function also sets both the accelerometer and the gyroscope
   * to their most sensitive settings, namely +/- 2g and +/- 250 degrees/sec, and sets
   * the clock source to use the X Gyro for reference, which is slightly better than
   * the default internal clock source.
   */
  pub fn initialize(&mut self) -> Result<()> {
    self.set_clock_source(CLOCK_PLL_XGYRO);
    return Ok(());
  }

  /** Verify the I2C connection.
   * Make sure the device is connected and responds as expected.
   * @return True if connection is valid, false otherwise
   */
  pub fn test_connection(&mut self) -> bool {
    match self.get_device_id() { 
        Ok(val) => return val == 0x71,
        Err(_) => return false,
    }
  }

  // AUX_VDDIO register (InvenSense demo code calls this RA_*G_OFFS_TC)
  
  /** Get the auxiliary I2C supply voltage level.
   * When set to 1, the auxiliary I2C bus high logic level is VDD. When cleared to
   * 0, the auxiliary I2C bus high logic level is VLOGIC. This does not apply to
   * the MPU-6000, which does not have a VLOGIC pin.
   * @return I2C supply voltage level (0=VLOGIC, 1=VDD)
   */
  pub fn get_aux_vddio_level(&mut self) -> Result<u8> {
    return i2c::read_bit(self.dev_address, RA_YG_OFFS_TC, TC_PWR_MODE_BIT);
  }

  /** Set the auxiliary I2C supply voltage level.
   * When set to 1, the auxiliary I2C bus high logic level is VDD. When cleared to
   * 0, the auxiliary I2C bus high logic level is VLOGIC. This does not apply to
   * the MPU-6000, which does not have a VLOGIC pin.
   * @param level I2C supply voltage level (0=VLOGIC, 1=VDD)
   */
  pub fn set_aux_vddio_level(&mut self, enabled : bool) -> Result<()> {
    return i2c::write_bit(self.dev_address, RA_YG_OFFS_TC, TC_PWR_MODE_BIT, enabled as u8);
  }

  // SMPLRT_DIV register

  /** Get gyroscope output rate divider.
   * The sensor register output, FIFO output, DMP sampling, Motion detection, Zero
   * Motion detection, and Free Fall detection are all based on the Sample Rate.
   * The Sample Rate is generated by dividing the gyroscope output rate by
   * SMPLRT_DIV:
   *
   * Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
   *
   * where Gyroscope Output Rate = 8kHz when the DLPF is disabled (DLPF_CFG = 0 or
   * 7), and 1kHz when the DLPF is enabled (see Register 26).
   *
   * Note: The accelerometer output rate is 1kHz. This means that for a Sample
   * Rate greater than 1kHz, the same accelerometer sample may be output to the
   * FIFO, DMP, and sensor registers more than once.
   *
   * For a diagram of the gyroscope and accelerometer signal paths, see Section 8
   * of the MPU-6000/MPU-9250 Product Specification document.
   *
   * @return Current sample rate
   * @see MPU9250_RA_SMPLRT_DIV
   */
  pub fn get_rate(&mut self) -> Result<u8> {
    return i2c::read_byte(self.dev_address, RA_SMPLRT_DIV);
  }

  /** Set gyroscope sample rate divider.
   * @param rate New sample rate divider
   * @see getRate()
   * @see MPU9250_RA_SMPLRT_DIV
   */
  pub fn set_rate(&mut self, rate : u8) -> Result<()> {
    return i2c::write_byte(self.dev_address, RA_SMPLRT_DIV, rate);
  }


  // CONFIG register
  
  /** Get external FSYNC configuration.
   * Configures the external Frame Synchronization (FSYNC) pin sampling. An
   * external signal connected to the FSYNC pin can be sampled by configuring
   * EXT_SYNC_SET. Signal changes to the FSYNC pin are latched so that short
   * strobes may be captured. The latched FSYNC signal will be sampled at the
   * Sampling Rate, as defined in register 25. After sampling, the latch will
   * reset to the current FSYNC signal state.
   *
   * The sampled value will be reported in place of the least significant bit in
   * a sensor data register determined by the value of EXT_SYNC_SET according to
   * the following table.
   *
   * <pre>
   * EXT_SYNC_SET | FSYNC Bit Location
   * -------------+-------------------
   * 0            | Input disabled
   * 1            | TEMP_OUT_L[0]
   * 2            | GYRO_XOUT_L[0]
   * 3            | GYRO_YOUT_L[0]
   * 4            | GYRO_ZOUT_L[0]
   * 5            | ACCEL_XOUT_L[0]
   * 6            | ACCEL_YOUT_L[0]
   * 7            | ACCEL_ZOUT_L[0]
   * </pre>
   *
   * @return FSYNC configuration value
   */
  pub fn get_external_frame_sync(&mut self) -> Result<u8> {
    return i2c::read_bits(self.dev_address, RA_CONFIG, CFG_EXT_SYNC_SET_BIT, CFG_EXT_SYNC_SET_LENGTH);
  }

  /** Set external FSYNC configuration.
   * @see getExternalFrameSync()
   * @see MPU9250_RA_CONFIG
   * @param sync New FSYNC configuration value
   */
  pub fn set_external_frame_sync(&mut self, sync : u8) -> Result<()> {
    return i2c::write_bits(self.dev_address, RA_CONFIG, CFG_EXT_SYNC_SET_BIT, CFG_EXT_SYNC_SET_LENGTH, sync);
  }


  /** Get digital low-pass filter configuration.
   * The DLPF_CFG parameter sets the digital low pass filter configuration. It
   * also determines the internal sampling rate used by the device as shown in
   * the table below.
   *
   * Note: The accelerometer output rate is 1kHz. This means that for a Sample
   * Rate greater than 1kHz, the same accelerometer sample may be output to the
   * FIFO, DMP, and sensor registers more than once.
   *
   * <pre>
   *          |   ACCELEROMETER    |           GYROSCOPE
   * DLPF_CFG | Bandwidth | Delay  | Bandwidth | Delay  | Sample Rate
   * ---------+-----------+--------+-----------+--------+-------------
   * 0        | 260Hz     | 0ms    | 256Hz     | 0.98ms | 8kHz
   * 1        | 184Hz     | 2.0ms  | 188Hz     | 1.9ms  | 1kHz
   * 2        | 94Hz      | 3.0ms  | 98Hz      | 2.8ms  | 1kHz
   * 3        | 44Hz      | 4.9ms  | 42Hz      | 4.8ms  | 1kHz
   * 4        | 21Hz      | 8.5ms  | 20Hz      | 8.3ms  | 1kHz
   * 5        | 10Hz      | 13.8ms | 10Hz      | 13.4ms | 1kHz
   * 6        | 5Hz       | 19.0ms | 5Hz       | 18.6ms | 1kHz
   * 7        |   -- Reserved --   |   -- Reserved --   | Reserved
   * </pre>
   *
   * @return DLFP configuration
   * @see MPU9250_RA_CONFIG
   * @see MPU9250_CFG_DLPF_CFG_BIT
   * @see MPU9250_CFG_DLPF_CFG_LENGTH
   */
  pub fn get_dlpf_mode(&mut self) -> Result<u8> {
    return i2c::read_bits(self.dev_address, RA_CONFIG, CFG_DLPF_CFG_BIT, CFG_DLPF_CFG_LENGTH);
  }
  /** Set digital low-pass filter configuration.
   * @param mode New DLFP configuration setting
   * @see getDLPFBandwidth()
   * @see MPU9250_DLPF_BW_256
   * @see MPU9250_RA_CONFIG
   * @see MPU9250_CFG_DLPF_CFG_BIT
   * @see MPU9250_CFG_DLPF_CFG_LENGTH
   */
  pub fn set_dlpf_mode(&mut self, mode : u8) -> Result<()> {
    return i2c::write_bits(self.dev_address, RA_CONFIG, CFG_DLPF_CFG_BIT, CFG_DLPF_CFG_LENGTH, mode);
  }

  // GYRO_CONFIG register
  
  /** Get full-scale gyroscope range.
   * The FS_SEL parameter allows setting the full-scale range of the gyro sensors,
   * as described in the table below.
   *
   * <pre>
   * 0 = +/- 250 degrees/sec
   * 1 = +/- 500 degrees/sec
   * 2 = +/- 1000 degrees/sec
   * 3 = +/- 2000 degrees/sec
   * </pre>
   *
   * @return Current full-scale gyroscope range setting
   * @see MPU9250_GYRO_FS_250
   * @see MPU9250_RA_GYRO_CONFIG
   * @see MPU9250_GCONFIG_FS_SEL_BIT
   * @see MPU9250_GCONFIG_FS_SEL_LENGTH
   */
  pub fn get_full_scale_gyro_range(&mut self) -> Result<u8> {
    return i2c::read_bits(self.dev_address, RA_GYRO_CONFIG, GCONFIG_FS_SEL_BIT, GCONFIG_FS_SEL_LENGTH);
  }
  /** Set full-scale gyroscope range.
   * @param range New full-scale gyroscope range value
   * @see getFullScaleRange()
   * @see MPU9250_GYRO_FS_250
   * @see MPU9250_RA_GYRO_CONFIG
   * @see MPU9250_GCONFIG_FS_SEL_BIT
   * @see MPU9250_GCONFIG_FS_SEL_LENGTH
   */
  pub fn set_full_scale_gyro_range(&mut self, range : u8) -> Result<()> {
    return i2c::write_bits(self.dev_address, RA_GYRO_CONFIG, GCONFIG_FS_SEL_BIT, GCONFIG_FS_SEL_LENGTH, range);
  }


  // ACCEL_CONFIG register
  
  /** Get self-test enabled setting for accelerometer X axis.
   * @return Self-test enabled value
   * @see MPU9250_RA_ACCEL_CONFIG
   */
  pub fn get_accel_x_self_test(&mut self) -> Result<u8> {
    return i2c::read_bit(self.dev_address, RA_ACCEL_CONFIG, ACONFIG_XA_ST_BIT);
  }
  /** Get self-test enabled setting for accelerometer X axis.
   * @param enabled Self-test enabled value
   * @see MPU9250_RA_ACCEL_CONFIG
   */
  pub fn set_accel_x_self_test(&mut self, enabled : bool) -> Result<()> {
    return i2c::write_bit(self.dev_address, RA_ACCEL_CONFIG, ACONFIG_XA_ST_BIT, enabled as u8);
  }

  /** Get self-test enabled value for accelerometer Y axis.
   * @return Self-test enabled value
   * @see MPU9250_RA_ACCEL_CONFIG
   */
  pub fn get_accel_y_self_test(&mut self) -> Result<u8> {
    return i2c::read_bit(self.dev_address, RA_ACCEL_CONFIG, ACONFIG_YA_ST_BIT);
  }
  /** Get self-test enabled value for accelerometer Y axis.
   * @param enabled Self-test enabled value
   * @see MPU9250_RA_ACCEL_CONFIG
   */
  pub fn set_accel_y_self_test(&mut self, enabled : bool) -> Result<()> {
    return i2c::write_bit(self.dev_address, RA_ACCEL_CONFIG, ACONFIG_YA_ST_BIT, enabled as u8);
  }
  /** Get self-test enabled value for accelerometer Z axis.
   * @return Self-test enabled value
   * @see MPU9250_RA_ACCEL_CONFIG
   */
  pub fn get_accel_z_self_test(&mut self) -> Result<u8> {
    return i2c::read_bit(self.dev_address, RA_ACCEL_CONFIG, ACONFIG_ZA_ST_BIT);
  }
  /** Set self-test enabled value for accelerometer Z axis.
   * @param enabled Self-test enabled value
   * @see MPU9250_RA_ACCEL_CONFIG
   */
  pub fn set_accel_z_self_test(&mut self, enabled : bool) -> Result<()> {
    return i2c::write_bit(self.dev_address, RA_ACCEL_CONFIG, ACONFIG_ZA_ST_BIT, enabled as u8);
  }
  /** Get full-scale accelerometer range.
   * The FS_SEL parameter allows setting the full-scale range of the accelerometer
   * sensors, as described in the table below.
   *
   * <pre>
   * 0 = +/- 2g
   * 1 = +/- 4g
   * 2 = +/- 8g
   * 3 = +/- 16g
   * </pre>
   *
   * @return Current full-scale accelerometer range setting
   * @see MPU9250_ACCEL_FS_2
   * @see MPU9250_RA_ACCEL_CONFIG
   * @see MPU9250_ACONFIG_AFS_SEL_BIT
   * @see MPU9250_ACONFIG_AFS_SEL_LENGTH
   */
  pub fn get_full_scale_accel_range(&mut self) -> Result<u8> {
    return i2c::read_bits(self.dev_address, RA_ACCEL_CONFIG, ACONFIG_AFS_SEL_BIT, ACONFIG_AFS_SEL_LENGTH);
  }
  /** Set full-scale accelerometer range.
   * @param range New full-scale accelerometer range setting
   * @see getFullScaleAccelRange()
   */
  pub fn set_full_scale_accel_range(&mut self, range : u8) -> Result<()> {
    return i2c::write_bits(self.dev_address, RA_ACCEL_CONFIG, ACONFIG_AFS_SEL_BIT, ACONFIG_AFS_SEL_LENGTH, range);
  }
  /** Get the high-pass filter configuration.
   * The DHPF is a filter module in the path leading to motion detectors (Free
   * Fall, Motion threshold, and Zero Motion). The high pass filter output is not
   * available to the data registers (see Figure in Section 8 of the MPU-6000/
   * MPU-9250 Product Specification document).
   *
   * The high pass filter has three modes:
   *
   * <pre>
   *    Reset: The filter output settles to zero within one sample. This
   *           effectively disables the high pass filter. This mode may be toggled
   *           to quickly settle the filter.
   *
   *    On:    The high pass filter will pass signals above the cut off frequency.
   *
   *    Hold:  When triggered, the filter holds the present sample. The filter
   *           output will be the difference between the input sample and the held
   *           sample.
   * </pre>
   *
   * <pre>
   * ACCEL_HPF | Filter Mode | Cut-off Frequency
   * ----------+-------------+------------------
   * 0         | Reset       | None
   * 1         | On          | 5Hz
   * 2         | On          | 2.5Hz
   * 3         | On          | 1.25Hz
   * 4         | On          | 0.63Hz
   * 7         | Hold        | None
   * </pre>
   *
   * @return Current high-pass filter configuration
   * @see MPU9250_DHPF_RESET
   * @see MPU9250_RA_ACCEL_CONFIG
   */
  pub fn get_dhpf_mode(&mut self) -> Result<u8> {
    return i2c::read_bits(self.dev_address, RA_ACCEL_CONFIG, ACONFIG_ACCEL_HPF_BIT, ACONFIG_ACCEL_HPF_LENGTH);
  }
  /** Set the high-pass filter configuration.
   * @param bandwidth New high-pass filter configuration
   * @see setDHPFMode()
   * @see MPU9250_DHPF_RESET
   * @see MPU9250_RA_ACCEL_CONFIG
   */
  pub fn set_dhpf_mode(&mut self, mode : u8) -> Result<()> {
    return i2c::write_bits(self.dev_address, RA_ACCEL_CONFIG, ACONFIG_ACCEL_HPF_BIT, ACONFIG_ACCEL_HPF_LENGTH, mode);
  }

  // FF_THR register
  
  /** Get free-fall event acceleration threshold.
   * This register configures the detection threshold for Free Fall event
   * detection. The unit of FF_THR is 1LSB = 2mg. Free Fall is detected when the
   * absolute value of the accelerometer measurements for the three axes are each
   * less than the detection threshold. This condition increments the Free Fall
   * duration counter (Register 30). The Free Fall interrupt is triggered when the
   * Free Fall duration counter reaches the time specified in FF_DUR.
   *
   * For more details on the Free Fall detection interrupt, see Section 8.2 of the
   * MPU-6000/MPU-9250 Product Specification document as well as Registers 56 and
   * 58 of this document.
   *
   * @return Current free-fall acceleration threshold value (LSB = 2mg)
   * @see MPU9250_RA_FF_THR
   */
  pub fn get_free_fall_detection_threshold(&mut self) -> Result<u8> {
    return i2c::read_byte(self.dev_address, RA_FF_THR);
  }
  /** Get free-fall event acceleration threshold.
   * @param threshold New free-fall acceleration threshold value (LSB = 2mg)
   * @see getFreefallDetectionThreshold()
   * @see MPU9250_RA_FF_THR
   */
  pub fn set_free_fall_detection_threshold(&mut self, threshold : u8) -> Result<()> {
    return i2c::write_byte(self.dev_address, RA_FF_THR, threshold);
  }
  
  // FF_DUR register
  
  /** Get free-fall event duration threshold.
   * This register configures the duration counter threshold for Free Fall event
   * detection. The duration counter ticks at 1kHz, therefore FF_DUR has a unit
   * of 1 LSB = 1 ms.
   *
   * The Free Fall duration counter increments while the absolute value of the
   * accelerometer measurements are each less than the detection threshold
   * (Register 29). The Free Fall interrupt is triggered when the Free Fall
   * duration counter reaches the time specified in this register.
   *
   * For more details on the Free Fall detection interrupt, see Section 8.2 of
   * the MPU-6000/MPU-9250 Product Specification document as well as Registers 56
   * and 58 of this document.
   *
   * @return Current free-fall duration threshold value (LSB = 1ms)
   * @see MPU9250_RA_FF_DUR
   */
  pub fn get_free_fall_detection_duration(&mut self) -> Result<u8> {
    return i2c::read_byte(self.dev_address, RA_FF_DUR);
  }
  /** Get free-fall event duration threshold.
   * @param duration New free-fall duration threshold value (LSB = 1ms)
   * @see getFreefallDetectionDuration()
   * @see MPU9250_RA_FF_DUR
   */
  pub fn set_free_fall_detection_duration(&mut self, duration : u8) -> Result<()> {
    return i2c::write_byte(self.dev_address, RA_FF_DUR, duration);
  }
  

  // MOT_THR register
  
  /** Get motion detection event acceleration threshold.
   * This register configures the detection threshold for Motion interrupt
   * generation. The unit of MOT_THR is 1LSB = 2mg. Motion is detected when the
   * absolute value of any of the accelerometer measurements exceeds this Motion
   * detection threshold. This condition increments the Motion detection duration
   * counter (Register 32). The Motion detection interrupt is triggered when the
   * Motion Detection counter reaches the time count specified in MOT_DUR
   * (Register 32).
   *
   * The Motion interrupt will indicate the axis and polarity of detected motion
   * in MOT_DETECT_STATUS (Register 97).
   *
   * For more details on the Motion detection interrupt, see Section 8.3 of the
   * MPU-6000/MPU-9250 Product Specification document as well as Registers 56 and
   * 58 of this document.
   *
   * @return Current motion detection acceleration threshold value (LSB = 2mg)
   * @see MPU9250_RA_MOT_THR
   */
  pub fn get_motion_detection_threshold(&mut self) -> Result<u8> {
    return i2c::read_byte(self.dev_address, RA_MOT_THR);
  }
  /** Set free-fall event acceleration threshold.
   * @param threshold New motion detection acceleration threshold value (LSB = 2mg)
   * @see getMotionDetectionThreshold()
   * @see MPU9250_RA_MOT_THR
   */
  pub fn set_motion_detection_threshold(&mut self, threshold : u8) -> Result<()> {
    return i2c::write_byte(self.dev_address, RA_MOT_THR, threshold);
  }
  
  // MOT_DUR register
  
  /** Get motion detection event duration threshold.
   * This register configures the duration counter threshold for Motion interrupt
   * generation. The duration counter ticks at 1 kHz, therefore MOT_DUR has a unit
   * of 1LSB = 1ms. The Motion detection duration counter increments when the
   * absolute value of any of the accelerometer measurements exceeds the Motion
   * detection threshold (Register 31). The Motion detection interrupt is
   * triggered when the Motion detection counter reaches the time count specified
   * in this register.
   *
   * For more details on the Motion detection interrupt, see Section 8.3 of the
   * MPU-6000/MPU-9250 Product Specification document.
   *
   * @return Current motion detection duration threshold value (LSB = 1ms)
   * @see MPU9250_RA_MOT_DUR
   */
  pub fn get_motion_detection_duration(&mut self) -> Result<u8> {
    return i2c::read_byte(self.dev_address, RA_MOT_DUR);
  }
  /** Set motion detection event duration threshold.
   * @param duration New motion detection duration threshold value (LSB = 1ms)
   * @see getMotionDetectionDuration()
   * @see MPU9250_RA_MOT_DUR
   */
  pub fn set_motion_detection_duration(&mut self, duration : u8) -> Result<()> {
    return i2c::write_byte(self.dev_address, RA_MOT_DUR, duration);
  }

  // ZRMOT_THR register
  
  /** Get zero motion detection event acceleration threshold.
   * This register configures the detection threshold for Zero Motion interrupt
   * generation. The unit of ZRMOT_THR is 1LSB = 2mg. Zero Motion is detected when
   * the absolute value of the accelerometer measurements for the 3 axes are each
   * less than the detection threshold. This condition increments the Zero Motion
   * duration counter (Register 34). The Zero Motion interrupt is triggered when
   * the Zero Motion duration counter reaches the time count specified in
   * ZRMOT_DUR (Register 34).
   *
   * Unlike Free Fall or Motion detection, Zero Motion detection triggers an
   * interrupt both when Zero Motion is first detected and when Zero Motion is no
   * longer detected.
   *
   * When a zero motion event is detected, a Zero Motion Status will be indicated
   * in the MOT_DETECT_STATUS register (Register 97). When a motion-to-zero-motion
   * condition is detected, the status bit is set to 1. When a zero-motion-to-
   * motion condition is detected, the status bit is set to 0.
   *
   * For more details on the Zero Motion detection interrupt, see Section 8.4 of
   * the MPU-6000/MPU-9250 Product Specification document as well as Registers 56
   * and 58 of this document.
   *
   * @return Current zero motion detection acceleration threshold value (LSB = 2mg)
   * @see MPU9250_RA_ZRMOT_THR
   */
  pub fn get_zero_motion_detection_threshold(&mut self) -> Result<u8> {
    return i2c::read_byte(self.dev_address, RA_ZRMOT_THR);
  }
  /** Set zero motion detection event acceleration threshold.
   * @param threshold New zero motion detection acceleration threshold value (LSB = 2mg)
   * @see getZeroMotionDetectionThreshold()
   * @see MPU9250_RA_ZRMOT_THR
   */
  pub fn set_zero_motion_detection_threshold(&mut self, threshold : u8) -> Result<()> {
    return i2c::write_byte(self.dev_address, RA_ZRMOT_THR, threshold);
  }
  
  // ZRMOT_DUR register
  
  /** Get zero motion detection event duration threshold.
   * This register configures the duration counter threshold for Zero Motion
   * interrupt generation. The duration counter ticks at 16 Hz, therefore
   * ZRMOT_DUR has a unit of 1 LSB = 64 ms. The Zero Motion duration counter
   * increments while the absolute value of the accelerometer measurements are
   * each less than the detection threshold (Register 33). The Zero Motion
   * interrupt is triggered when the Zero Motion duration counter reaches the time
   * count specified in this register.
   *
   * For more details on the Zero Motion detection interrupt, see Section 8.4 of
   * the MPU-6000/MPU-9250 Product Specification document, as well as Registers 56
   * and 58 of this document.
   *
   * @return Current zero motion detection duration threshold value (LSB = 64ms)
   * @see MPU9250_RA_ZRMOT_DUR
   */
  pub fn get_zero_motion_detection_duration(&mut self) -> Result<u8> {
    return i2c::read_byte(self.dev_address, RA_ZRMOT_DUR);
  }
  /** Set zero motion detection event duration threshold.
   * @param duration New zero motion detection duration threshold value (LSB = 1ms)
   * @see getZeroMotionDetectionDuration()
   * @see MPU9250_RA_ZRMOT_DUR
   */
  pub fn set_zero_motion_detection_duration(&mut self, duration : u8) -> Result<()> {
    return i2c::write_byte(self.dev_address, RA_ZRMOT_DUR, duration);
  }


  // FIFO_EN register
  
  /** Get temperature FIFO enabled value.
   * When set to 1, this bit enables TEMP_OUT_H and TEMP_OUT_L (Registers 65 and
   * 66) to be written into the FIFO buffer.
   * @return Current temperature FIFO enabled value
   * @see MPU9250_RA_FIFO_EN
   */
  pub fn get_temp_fifo_enabled(&mut self) -> Result<u8> {
    return i2c::read_bit(self.dev_address, RA_FIFO_EN, TEMP_FIFO_EN_BIT);
  }
  /** Set temperature FIFO enabled value.
   * @param enabled New temperature FIFO enabled value
   * @see getTempFIFOEnabled()
   * @see MPU9250_RA_FIFO_EN
   */
  pub fn set_temp_fifo_enabled(&mut self, enabled : bool) -> Result<()> {
    return i2c::write_bit(self.dev_address, RA_FIFO_EN, TEMP_FIFO_EN_BIT, enabled as u8);
  }
  /** Get gyroscope X-axis FIFO enabled value.
   * When set to 1, this bit enables GYRO_XOUT_H and GYRO_XOUT_L (Registers 67 and
   * 68) to be written into the FIFO buffer.
   * @return Current gyroscope X-axis FIFO enabled value
   * @see MPU9250_RA_FIFO_EN
   */
  pub fn get_x_gyro_fifo_enabled(&mut self) -> Result<u8> {
    return i2c::read_bit(self.dev_address, RA_FIFO_EN, XG_FIFO_EN_BIT);
  }
  /** Set gyroscope X-axis FIFO enabled value.
   * @param enabled New gyroscope X-axis FIFO enabled value
   * @see getXGyroFIFOEnabled()
   * @see MPU9250_RA_FIFO_EN
   */
  pub fn set_x_gyro_fifo_enabled(&mut self, enabled : bool) -> Result<()> {
    return i2c::write_bit(self.dev_address, RA_FIFO_EN, XG_FIFO_EN_BIT, enabled as u8);
  }
  /** Get gyroscope Y-axis FIFO enabled value.
   * When set to 1, this bit enables GYRO_YOUT_H and GYRO_YOUT_L (Registers 69 and
   * 70) to be written into the FIFO buffer.
   * @return Current gyroscope Y-axis FIFO enabled value
   * @see MPU9250_RA_FIFO_EN
   */
  pub fn get_y_gyro_fifo_enabled(&mut self) -> Result<u8> {
    return i2c::read_bit(self.dev_address, RA_FIFO_EN, YG_FIFO_EN_BIT);
  }
  /** Set gyroscope Y-axis FIFO enabled value.
   * @param enabled New gyroscope Y-axis FIFO enabled value
   * @see getYGyroFIFOEnabled()
   * @see MPU9250_RA_FIFO_EN
   */
  pub fn set_y_gyro_fifo_enabled(&mut self, enabled : bool) -> Result<()> {
    return i2c::write_bit(self.dev_address, RA_FIFO_EN, YG_FIFO_EN_BIT, enabled as u8);
  }
  /** Get gyroscope Z-axis FIFO enabled value.
   * When set to 1, this bit enables GYRO_ZOUT_H and GYRO_ZOUT_L (Registers 71 and
   * 72) to be written into the FIFO buffer.
   * @return Current gyroscope Z-axis FIFO enabled value
   * @see MPU9250_RA_FIFO_EN
   */
  pub fn get_z_gyro_fifo_enabled(&mut self) -> Result<u8> {
    return i2c::read_bit(self.dev_address, RA_FIFO_EN, ZG_FIFO_EN_BIT);
  }
  /** Set gyroscope Z-axis FIFO enabled value.
   * @param enabled New gyroscope Z-axis FIFO enabled value
   * @see getZGyroFIFOEnabled()
   * @see MPU9250_RA_FIFO_EN
   */
  pub fn set_z_gyro_fifo_enabled(&mut self, enabled : bool) -> Result<()> {
    return i2c::write_bit(self.dev_address, RA_FIFO_EN, ZG_FIFO_EN_BIT, enabled as u8);
  }
  /** Get accelerometer FIFO enabled value.
   * When set to 1, this bit enables ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H,
   * ACCEL_YOUT_L, ACCEL_ZOUT_H, and ACCEL_ZOUT_L (Registers 59 to 64) to be
   * written into the FIFO buffer.
   * @return Current accelerometer FIFO enabled value
   * @see MPU9250_RA_FIFO_EN
   */
  pub fn get_accel_fifo_enabled(&mut self) -> Result<u8> {
    return i2c::read_bit(self.dev_address, RA_FIFO_EN, ACCEL_FIFO_EN_BIT);
  }
  /** Set accelerometer FIFO enabled value.
   * @param enabled New accelerometer FIFO enabled value
   * @see getAccelFIFOEnabled()
   * @see MPU9250_RA_FIFO_EN
   */
  pub fn set_accel_fifo_enabled(&mut self, enabled : bool) -> Result<()> {
    return i2c::write_bit(self.dev_address, RA_FIFO_EN, ACCEL_FIFO_EN_BIT, enabled as u8);
  }
  /** Get Slave 2 FIFO enabled value.
   * When set to 1, this bit enables EXT_SENS_DATA registers (Registers 73 to 96)
   * associated with Slave 2 to be written into the FIFO buffer.
   * @return Current Slave 2 FIFO enabled value
   * @see MPU9250_RA_FIFO_EN
   */
  pub fn get_slave2_fifo_enabled(&mut self) -> Result<u8> {
    return i2c::read_bit(self.dev_address, RA_FIFO_EN, SLV2_FIFO_EN_BIT);
  }
  /** Set Slave 2 FIFO enabled value.
   * @param enabled New Slave 2 FIFO enabled value
   * @see getSlave2FIFOEnabled()
   * @see MPU9250_RA_FIFO_EN
   */
  pub fn set_slave2_fifo_enabled(&mut self, enabled : bool) -> Result<()> {
    return i2c::write_bit(self.dev_address, RA_FIFO_EN, SLV2_FIFO_EN_BIT, enabled as u8);
  }
  /** Get Slave 1 FIFO enabled value.
   * When set to 1, this bit enables EXT_SENS_DATA registers (Registers 73 to 96)
   * associated with Slave 1 to be written into the FIFO buffer.
   * @return Current Slave 1 FIFO enabled value
   * @see MPU9250_RA_FIFO_EN
   */
  pub fn get_slave1_fifo_enabled(&mut self) -> Result<u8> {
    return i2c::read_bit(self.dev_address, RA_FIFO_EN, SLV1_FIFO_EN_BIT);
  }
  /** Set Slave 1 FIFO enabled value.
   * @param enabled New Slave 1 FIFO enabled value
   * @see getSlave1FIFOEnabled()
   * @see MPU9250_RA_FIFO_EN
   */
  pub fn set_slave1_fifo_enabled(&mut self, enabled : bool) -> Result<()> {
    return i2c::write_bit(self.dev_address, RA_FIFO_EN, SLV1_FIFO_EN_BIT, enabled as u8);
  }
  /** Get Slave 0 FIFO enabled value.
   * When set to 1, this bit enables EXT_SENS_DATA registers (Registers 73 to 96)
   * associated with Slave 0 to be written into the FIFO buffer.
   * @return Current Slave 0 FIFO enabled value
   * @see MPU9250_RA_FIFO_EN
   */
  pub fn get_slave0_fifo_enabled(&mut self) -> Result<u8> {
    return i2c::read_bit(self.dev_address, RA_FIFO_EN, SLV0_FIFO_EN_BIT);
  }
  /** Set Slave 0 FIFO enabled value.
   * @param enabled New Slave 0 FIFO enabled value
   * @see getSlave0FIFOEnabled()
   * @see MPU9250_RA_FIFO_EN
   */
  pub fn set_slave0_fifo_enabled(&mut self, enabled : bool) -> Result<()> {
    return i2c::write_bit(self.dev_address, RA_FIFO_EN, SLV0_FIFO_EN_BIT, enabled as u8);
  }

  // I2C_MST_CTRL register
  
  /** Get multi-master enabled value.
   * Multi-master capability allows multiple I2C masters to operate on the same
   * bus. In circuits where multi-master capability is required, set MULT_MST_EN
   * to 1. This will increase current drawn by approximately 30uA.
   *
   * In circuits where multi-master capability is required, the state of the I2C
   * bus must always be monitored by each separate I2C Master. Before an I2C
   * Master can assume arbitration of the bus, it must first confirm that no other
   * I2C Master has arbitration of the bus. When MULT_MST_EN is set to 1, the
   * MPU-60X0's bus arbitration detection logic is turned on, enabling it to
   * detect when the bus is available.
   *
   * @return Current multi-master enabled value
   * @see MPU9250_RA_I2C_MST_CTRL
   */
  pub fn get_multi_master_enabled(&mut self) -> Result<u8> {
    return i2c::read_bit(self.dev_address, RA_I2C_MST_CTRL, MULT_MST_EN_BIT);
  }
  /** Set multi-master enabled value.
   * @param enabled New multi-master enabled value
   * @see getMultiMasterEnabled()
   * @see MPU9250_RA_I2C_MST_CTRL
   */
  pub fn set_multi_master_enabled(&mut self, enabled : bool) -> Result<()> {
    return i2c::write_bit(self.dev_address, RA_I2C_MST_CTRL, MULT_MST_EN_BIT, enabled as u8);
  }
  /** Get wait-for-external-sensor-data enabled value.
   * When the WAIT_FOR_ES bit is set to 1, the Data Ready interrupt will be
   * delayed until External Sensor data from the Slave Devices are loaded into the
   * EXT_SENS_DATA registers. This is used to ensure that both the internal sensor
   * data (i.e. from gyro and accel) and external sensor data have been loaded to
   * their respective data registers (i.e. the data is synced) when the Data Ready
   * interrupt is triggered.
   *
   * @return Current wait-for-external-sensor-data enabled value
   * @see MPU9250_RA_I2C_MST_CTRL
   */
  pub fn get_wait_for_external_sensor_enabled(&mut self) -> Result<u8> {
    return i2c::read_bit(self.dev_address, RA_I2C_MST_CTRL, WAIT_FOR_ES_BIT);
  }
  /** Set wait-for-external-sensor-data enabled value.
   * @param enabled New wait-for-external-sensor-data enabled value
   * @see getWaitForExternalSensorEnabled()
   * @see MPU9250_RA_I2C_MST_CTRL
   */
  pub fn set_wait_for_external_sensor_enabled(&mut self, enabled : bool) -> Result<()> {
    return i2c::write_bit(self.dev_address, RA_I2C_MST_CTRL, WAIT_FOR_ES_BIT, enabled as u8);
  }
  /** Get Slave 3 FIFO enabled value.
   * When set to 1, this bit enables EXT_SENS_DATA registers (Registers 73 to 96)
   * associated with Slave 3 to be written into the FIFO buffer.
   * @return Current Slave 3 FIFO enabled value
   * @see MPU9250_RA_MST_CTRL
   */
  pub fn get_slave3_fifo_enabled(&mut self) -> Result<u8> {
    return i2c::read_bit(self.dev_address, RA_I2C_MST_CTRL, SLV_3_FIFO_EN_BIT);
  }
  /** Set Slave 3 FIFO enabled value.
   * @param enabled New Slave 3 FIFO enabled value
   * @see getSlave3FIFOEnabled()
   * @see MPU9250_RA_MST_CTRL
   */
  pub fn set_slave3_fifo_enabled(&mut self, enabled : bool) -> Result<()> {
    return i2c::write_bit(self.dev_address, RA_I2C_MST_CTRL, SLV_3_FIFO_EN_BIT, enabled as u8);
  }
  /** Get slave read/write transition enabled value.
   * The I2C_MST_P_NSR bit configures the I2C Master's transition from one slave
   * read to the next slave read. If the bit equals 0, there will be a restart
   * between reads. If the bit equals 1, there will be a stop followed by a start
   * of the following read. When a write transaction follows a read transaction,
   * the stop followed by a start of the successive write will be always used.
   *
   * @return Current slave read/write transition enabled value
   * @see MPU9250_RA_I2C_MST_CTRL
   */
  pub fn get_slave_read_write_transition_enabled(&mut self) -> Result<u8> {
    return i2c::read_bit(self.dev_address, RA_I2C_MST_CTRL, I2C_MST_P_NSR_BIT);
  }
  /** Set slave read/write transition enabled value.
   * @param enabled New slave read/write transition enabled value
   * @see getSlaveReadWriteTransitionEnabled()
   * @see MPU9250_RA_I2C_MST_CTRL
   */
  pub fn set_slave_read_write_transition_enabled(&mut self, enabled : bool) -> Result<()> {
    return i2c::write_bit(self.dev_address, RA_I2C_MST_CTRL, I2C_MST_P_NSR_BIT, enabled as u8);
  }













  pub fn set_clock_source(&mut self, source : u8) -> Result<()> {
    return i2c::write_bits(self.dev_address, RA_PWR_MGMT_1, PWR1_CLKSEL_BIT, PWR1_CLKSEL_LENGTH, source);
  }


  pub fn set_sleep_enabled(&mut self, enabled : bool) -> Result<()> {
    return i2c::write_bit(self.dev_address, RA_PWR_MGMT_1, PWR1_SLEEP_BIT, enabled as u8);
  }

  pub fn get_device_id(&mut self) -> Result<u8> {
    return i2c::read_byte(self.dev_address, RA_WHO_AM_I);
  }

}

pub fn read_x_accelerometer() -> Result<u16> {

    let high = i2c::read_byte(DEFAULT_ADDRESS, RA_ACCEL_XOUT_H)?;
    let low = i2c::read_byte(DEFAULT_ADDRESS, RA_ACCEL_XOUT_L)?;
    let data : u16 = ((high as u16) << 8) + (low as u16);
    println!("Read ACCELEROMETER data: {}", data);
    return Ok(data);
}

