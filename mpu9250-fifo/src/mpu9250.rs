#![allow(unused)]

// I2Cdev library collection - MPU9250 I2C device class
// Based on InvenSense MPU-9250 register map document rev. 2.0, 5/19/2011 (RM-MPU-6000A-00)
// And on the C++ implementation by Jeff Rowberg <jeff@rowberg.net>
// Ported to Rust by Kyle Storey <kyle@kylona.com>
// This includes a substantial portion of Rowberg's work so the original license is included here.
// Other parts of the code base are licensed under GPL 2.0 as seen in LICENSE

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
use anyhow::anyhow;

pub mod mpu9150 {

    //Magnetometer Registers
    pub const RA_MAG_ADDRESS      : u16 = 0x0C;
    pub const RA_MAG_CTRL          : u8 = 0x0A;
    pub const RA_MAG_CTRL_SNGL_MSR : u8 = 0x01;
    pub const RA_MAG_XOUT_L        : u8 = 0x03;
    pub const RA_MAG_XOUT_H        : u8 = 0x04;
    pub const RA_MAG_YOUT_L        : u8 = 0x05;
    pub const RA_MAG_YOUT_H        : u8 = 0x06;
    pub const RA_MAG_ZOUT_L        : u8 = 0x07;
    pub const RA_MAG_ZOUT_H        : u8 = 0x08;

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
const CFG_FIFO_MODE_BIT : u8 = 5;
const CFG_FIFO_MODE_LENGTH : u8 = 1;
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


// ACCEL_*OUT_* registers
pub struct AccelerometerData {
  pub x: i16,
  pub y: i16,
  pub z: i16,
}
pub struct GyroscopeData {
  pub x: i16,
  pub y: i16,
  pub z: i16,
}
pub struct MagnetometerData {
  pub x: i16,
  pub y: i16,
  pub z: i16,
}


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
    self.set_clock_source(CLOCK_PLL_XGYRO)?;
    self.set_full_scale_gyro_range(GYRO_FS_250)?;
    self.set_full_scale_accel_range(ACCEL_FS_2)?;
    self.set_sleep_enabled(false)?;
    Ok(())
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
    i2c::read_bit(self.dev_address, RA_YG_OFFS_TC, TC_PWR_MODE_BIT)
  }

  /** Set the auxiliary I2C supply voltage level.
   * When set to 1, the auxiliary I2C bus high logic level is VDD. When cleared to
   * 0, the auxiliary I2C bus high logic level is VLOGIC. This does not apply to
   * the MPU-6000, which does not have a VLOGIC pin.
   * @param level I2C supply voltage level (0=VLOGIC, 1=VDD)
   */
  pub fn set_aux_vddio_level(&mut self, enabled : bool) -> Result<()> {
    i2c::write_bit(self.dev_address, RA_YG_OFFS_TC, TC_PWR_MODE_BIT, enabled as u8)
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
    i2c::read_byte(self.dev_address, RA_SMPLRT_DIV)
  }

  /** Set gyroscope sample rate divider.
   * @param rate New sample rate divider
   * @see getRate()
   * @see MPU9250_RA_SMPLRT_DIV
   */
  pub fn set_rate(&mut self, rate : u8) -> Result<()> {
    i2c::write_byte(self.dev_address, RA_SMPLRT_DIV, rate)
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
    i2c::read_bits(self.dev_address, RA_CONFIG, CFG_EXT_SYNC_SET_BIT, CFG_EXT_SYNC_SET_LENGTH)
  }

  /** Set external FSYNC configuration.
   * @see getExternalFrameSync()
   * @see MPU9250_RA_CONFIG
   * @param sync New FSYNC configuration value
   */
  pub fn set_external_frame_sync(&mut self, sync : u8) -> Result<()> {
    i2c::write_bits(self.dev_address, RA_CONFIG, CFG_EXT_SYNC_SET_BIT, CFG_EXT_SYNC_SET_LENGTH, sync)
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
    i2c::read_bits(self.dev_address, RA_CONFIG, CFG_DLPF_CFG_BIT, CFG_DLPF_CFG_LENGTH)
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
    i2c::write_bits(self.dev_address, RA_CONFIG, CFG_DLPF_CFG_BIT, CFG_DLPF_CFG_LENGTH, mode)
  }

  /** Get first in first out buffer mode configuration.
   * A 0 in this bit allows the buffer to overwrite old values when the fifo is full
   * A 1 in this bit prevents writes to the buffer when it is full
   * @see MPU9250_RA_CONFIG
   * @param mode New mode configuration value
   */
  pub fn get_fifo_mode(&mut self) -> Result<u8> {
    i2c::read_bits(self.dev_address, RA_CONFIG, CFG_FIFO_MODE_BIT, CFG_FIFO_MODE_LENGTH)
  }

  /** Set fifo mode configuration.
   * @see get_fifo_mode()
   * @see MPU9250_RA_CONFIG
   * @param mode New mode configuration value
   */
  pub fn set_fifo_mode(&mut self, mode : u8) -> Result<()> {
    i2c::write_bits(self.dev_address, RA_CONFIG, CFG_FIFO_MODE_BIT, CFG_FIFO_MODE_LENGTH, mode)
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
    i2c::read_bits(self.dev_address, RA_GYRO_CONFIG, GCONFIG_FS_SEL_BIT, GCONFIG_FS_SEL_LENGTH)
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
    i2c::write_bits(self.dev_address, RA_GYRO_CONFIG, GCONFIG_FS_SEL_BIT, GCONFIG_FS_SEL_LENGTH, range)
  }


  // ACCEL_CONFIG register
  
  /** Get self-test enabled setting for accelerometer X axis.
   * @return Self-test enabled value
   * @see MPU9250_RA_ACCEL_CONFIG
   */
  pub fn get_accel_x_self_test(&mut self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_ACCEL_CONFIG, ACONFIG_XA_ST_BIT)
  }
  /** Get self-test enabled setting for accelerometer X axis.
   * @param enabled Self-test enabled value
   * @see MPU9250_RA_ACCEL_CONFIG
   */
  pub fn set_accel_x_self_test(&mut self, enabled : bool) -> Result<()> {
    i2c::write_bit(self.dev_address, RA_ACCEL_CONFIG, ACONFIG_XA_ST_BIT, enabled as u8)
  }

  /** Get self-test enabled value for accelerometer Y axis.
   * @return Self-test enabled value
   * @see MPU9250_RA_ACCEL_CONFIG
   */
  pub fn get_accel_y_self_test(&mut self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_ACCEL_CONFIG, ACONFIG_YA_ST_BIT)
  }
  /** Get self-test enabled value for accelerometer Y axis.
   * @param enabled Self-test enabled value
   * @see MPU9250_RA_ACCEL_CONFIG
   */
  pub fn set_accel_y_self_test(&mut self, enabled : bool) -> Result<()> {
    i2c::write_bit(self.dev_address, RA_ACCEL_CONFIG, ACONFIG_YA_ST_BIT, enabled as u8)
  }
  /** Get self-test enabled value for accelerometer Z axis.
   * @return Self-test enabled value
   * @see MPU9250_RA_ACCEL_CONFIG
   */
  pub fn get_accel_z_self_test(&mut self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_ACCEL_CONFIG, ACONFIG_ZA_ST_BIT)
  }
  /** Set self-test enabled value for accelerometer Z axis.
   * @param enabled Self-test enabled value
   * @see MPU9250_RA_ACCEL_CONFIG
   */
  pub fn set_accel_z_self_test(&mut self, enabled : bool) -> Result<()> {
    i2c::write_bit(self.dev_address, RA_ACCEL_CONFIG, ACONFIG_ZA_ST_BIT, enabled as u8)
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
    i2c::read_bits(self.dev_address, RA_ACCEL_CONFIG, ACONFIG_AFS_SEL_BIT, ACONFIG_AFS_SEL_LENGTH)
  }
  /** Set full-scale accelerometer range.
   * @param range New full-scale accelerometer range setting
   * @see getFullScaleAccelRange()
   */
  pub fn set_full_scale_accel_range(&mut self, range : u8) -> Result<()> {
    i2c::write_bits(self.dev_address, RA_ACCEL_CONFIG, ACONFIG_AFS_SEL_BIT, ACONFIG_AFS_SEL_LENGTH, range)
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
    i2c::read_bits(self.dev_address, RA_ACCEL_CONFIG, ACONFIG_ACCEL_HPF_BIT, ACONFIG_ACCEL_HPF_LENGTH)
  }
  /** Set the high-pass filter configuration.
   * @param bandwidth New high-pass filter configuration
   * @see setDHPFMode()
   * @see MPU9250_DHPF_RESET
   * @see MPU9250_RA_ACCEL_CONFIG
   */
  pub fn set_dhpf_mode(&mut self, mode : u8) -> Result<()> {
    i2c::write_bits(self.dev_address, RA_ACCEL_CONFIG, ACONFIG_ACCEL_HPF_BIT, ACONFIG_ACCEL_HPF_LENGTH, mode)
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
    i2c::read_byte(self.dev_address, RA_FF_THR)
  }
  /** Get free-fall event acceleration threshold.
   * @param threshold New free-fall acceleration threshold value (LSB = 2mg)
   * @see getFreefallDetectionThreshold()
   * @see MPU9250_RA_FF_THR
   */
  pub fn set_free_fall_detection_threshold(&mut self, threshold : u8) -> Result<()> {
    i2c::write_byte(self.dev_address, RA_FF_THR, threshold)
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
    i2c::read_byte(self.dev_address, RA_FF_DUR)
  }
  /** Get free-fall event duration threshold.
   * @param duration New free-fall duration threshold value (LSB = 1ms)
   * @see getFreefallDetectionDuration()
   * @see MPU9250_RA_FF_DUR
   */
  pub fn set_free_fall_detection_duration(&mut self, duration : u8) -> Result<()> {
    i2c::write_byte(self.dev_address, RA_FF_DUR, duration)
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
    i2c::read_byte(self.dev_address, RA_MOT_THR)
  }
  /** Set free-fall event acceleration threshold.
   * @param threshold New motion detection acceleration threshold value (LSB = 2mg)
   * @see getMotionDetectionThreshold()
   * @see MPU9250_RA_MOT_THR
   */
  pub fn set_motion_detection_threshold(&mut self, threshold : u8) -> Result<()> {
    i2c::write_byte(self.dev_address, RA_MOT_THR, threshold)
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
    i2c::read_byte(self.dev_address, RA_MOT_DUR)
  }
  /** Set motion detection event duration threshold.
   * @param duration New motion detection duration threshold value (LSB = 1ms)
   * @see getMotionDetectionDuration()
   * @see MPU9250_RA_MOT_DUR
   */
  pub fn set_motion_detection_duration(&mut self, duration : u8) -> Result<()> {
    i2c::write_byte(self.dev_address, RA_MOT_DUR, duration)
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
    i2c::read_byte(self.dev_address, RA_ZRMOT_THR)
  }
  /** Set zero motion detection event acceleration threshold.
   * @param threshold New zero motion detection acceleration threshold value (LSB = 2mg)
   * @see getZeroMotionDetectionThreshold()
   * @see MPU9250_RA_ZRMOT_THR
   */
  pub fn set_zero_motion_detection_threshold(&mut self, threshold : u8) -> Result<()> {
    i2c::write_byte(self.dev_address, RA_ZRMOT_THR, threshold)
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
    i2c::read_byte(self.dev_address, RA_ZRMOT_DUR)
  }
  /** Set zero motion detection event duration threshold.
   * @param duration New zero motion detection duration threshold value (LSB = 1ms)
   * @see getZeroMotionDetectionDuration()
   * @see MPU9250_RA_ZRMOT_DUR
   */
  pub fn set_zero_motion_detection_duration(&mut self, duration : u8) -> Result<()> {
    i2c::write_byte(self.dev_address, RA_ZRMOT_DUR, duration)
  }


  // FIFO_EN register
  
  /** Get temperature FIFO enabled value.
   * When set to 1, this bit enables TEMP_OUT_H and TEMP_OUT_L (Registers 65 and
   * 66) to be written into the FIFO buffer.
   * @return Current temperature FIFO enabled value
   * @see MPU9250_RA_FIFO_EN
   */
  pub fn get_temp_fifo_enabled(&mut self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_FIFO_EN, TEMP_FIFO_EN_BIT)
  }
  /** Set temperature FIFO enabled value.
   * @param enabled New temperature FIFO enabled value
   * @see getTempFIFOEnabled()
   * @see MPU9250_RA_FIFO_EN
   */
  pub fn set_temp_fifo_enabled(&mut self, enabled : bool) -> Result<()> {
    i2c::write_bit(self.dev_address, RA_FIFO_EN, TEMP_FIFO_EN_BIT, enabled as u8)
  }
  /** Get gyroscope X-axis FIFO enabled value.
   * When set to 1, this bit enables GYRO_XOUT_H and GYRO_XOUT_L (Registers 67 and
   * 68) to be written into the FIFO buffer.
   * @return Current gyroscope X-axis FIFO enabled value
   * @see MPU9250_RA_FIFO_EN
   */
  pub fn get_x_gyro_fifo_enabled(&mut self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_FIFO_EN, XG_FIFO_EN_BIT)
  }
  /** Set gyroscope X-axis FIFO enabled value.
   * @param enabled New gyroscope X-axis FIFO enabled value
   * @see getXGyroFIFOEnabled()
   * @see MPU9250_RA_FIFO_EN
   */
  pub fn set_x_gyro_fifo_enabled(&mut self, enabled : bool) -> Result<()> {
    i2c::write_bit(self.dev_address, RA_FIFO_EN, XG_FIFO_EN_BIT, enabled as u8)
  }
  /** Get gyroscope Y-axis FIFO enabled value.
   * When set to 1, this bit enables GYRO_YOUT_H and GYRO_YOUT_L (Registers 69 and
   * 70) to be written into the FIFO buffer.
   * @return Current gyroscope Y-axis FIFO enabled value
   * @see MPU9250_RA_FIFO_EN
   */
  pub fn get_y_gyro_fifo_enabled(&mut self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_FIFO_EN, YG_FIFO_EN_BIT)
  }
  /** Set gyroscope Y-axis FIFO enabled value.
   * @param enabled New gyroscope Y-axis FIFO enabled value
   * @see getYGyroFIFOEnabled()
   * @see MPU9250_RA_FIFO_EN
   */
  pub fn set_y_gyro_fifo_enabled(&mut self, enabled : bool) -> Result<()> {
    i2c::write_bit(self.dev_address, RA_FIFO_EN, YG_FIFO_EN_BIT, enabled as u8)
  }
  /** Get gyroscope Z-axis FIFO enabled value.
   * When set to 1, this bit enables GYRO_ZOUT_H and GYRO_ZOUT_L (Registers 71 and
   * 72) to be written into the FIFO buffer.
   * @return Current gyroscope Z-axis FIFO enabled value
   * @see MPU9250_RA_FIFO_EN
   */
  pub fn get_z_gyro_fifo_enabled(&mut self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_FIFO_EN, ZG_FIFO_EN_BIT)
  }
  /** Set gyroscope Z-axis FIFO enabled value.
   * @param enabled New gyroscope Z-axis FIFO enabled value
   * @see getZGyroFIFOEnabled()
   * @see MPU9250_RA_FIFO_EN
   */
  pub fn set_z_gyro_fifo_enabled(&mut self, enabled : bool) -> Result<()> {
    i2c::write_bit(self.dev_address, RA_FIFO_EN, ZG_FIFO_EN_BIT, enabled as u8)
  }
  /** Get accelerometer FIFO enabled value.
   * When set to 1, this bit enables ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H,
   * ACCEL_YOUT_L, ACCEL_ZOUT_H, and ACCEL_ZOUT_L (Registers 59 to 64) to be
   * written into the FIFO buffer.
   * @return Current accelerometer FIFO enabled value
   * @see MPU9250_RA_FIFO_EN
   */
  pub fn get_accel_fifo_enabled(&mut self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_FIFO_EN, ACCEL_FIFO_EN_BIT)
  }
  /** Set accelerometer FIFO enabled value.
   * @param enabled New accelerometer FIFO enabled value
   * @see getAccelFIFOEnabled()
   * @see MPU9250_RA_FIFO_EN
   */
  pub fn set_accel_fifo_enabled(&mut self, enabled : bool) -> Result<()> {
    i2c::write_bit(self.dev_address, RA_FIFO_EN, ACCEL_FIFO_EN_BIT, enabled as u8)
  }
  /** Get Slave 2 FIFO enabled value.
   * When set to 1, this bit enables EXT_SENS_DATA registers (Registers 73 to 96)
   * associated with Slave 2 to be written into the FIFO buffer.
   * @return Current Slave 2 FIFO enabled value
   * @see MPU9250_RA_FIFO_EN
   */
  pub fn get_slave2_fifo_enabled(&mut self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_FIFO_EN, SLV2_FIFO_EN_BIT)
  }
  /** Set Slave 2 FIFO enabled value.
   * @param enabled New Slave 2 FIFO enabled value
   * @see getSlave2FIFOEnabled()
   * @see MPU9250_RA_FIFO_EN
   */
  pub fn set_slave2_fifo_enabled(&mut self, enabled : bool) -> Result<()> {
    i2c::write_bit(self.dev_address, RA_FIFO_EN, SLV2_FIFO_EN_BIT, enabled as u8)
  }
  /** Get Slave 1 FIFO enabled value.
   * When set to 1, this bit enables EXT_SENS_DATA registers (Registers 73 to 96)
   * associated with Slave 1 to be written into the FIFO buffer.
   * @return Current Slave 1 FIFO enabled value
   * @see MPU9250_RA_FIFO_EN
   */
  pub fn get_slave1_fifo_enabled(&mut self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_FIFO_EN, SLV1_FIFO_EN_BIT)
  }
  /** Set Slave 1 FIFO enabled value.
   * @param enabled New Slave 1 FIFO enabled value
   * @see getSlave1FIFOEnabled()
   * @see MPU9250_RA_FIFO_EN
   */
  pub fn set_slave1_fifo_enabled(&mut self, enabled : bool) -> Result<()> {
    i2c::write_bit(self.dev_address, RA_FIFO_EN, SLV1_FIFO_EN_BIT, enabled as u8)
  }
  /** Get Slave 0 FIFO enabled value.
   * When set to 1, this bit enables EXT_SENS_DATA registers (Registers 73 to 96)
   * associated with Slave 0 to be written into the FIFO buffer.
   * @return Current Slave 0 FIFO enabled value
   * @see MPU9250_RA_FIFO_EN
   */
  pub fn get_slave0_fifo_enabled(&mut self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_FIFO_EN, SLV0_FIFO_EN_BIT)
  }
  /** Set Slave 0 FIFO enabled value.
   * @param enabled New Slave 0 FIFO enabled value
   * @see getSlave0FIFOEnabled()
   * @see MPU9250_RA_FIFO_EN
   */
  pub fn set_slave0_fifo_enabled(&mut self, enabled : bool) -> Result<()> {
    i2c::write_bit(self.dev_address, RA_FIFO_EN, SLV0_FIFO_EN_BIT, enabled as u8)
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
    i2c::read_bit(self.dev_address, RA_I2C_MST_CTRL, MULT_MST_EN_BIT)
  }
  /** Set multi-master enabled value.
   * @param enabled New multi-master enabled value
   * @see getMultiMasterEnabled()
   * @see MPU9250_RA_I2C_MST_CTRL
   */
  pub fn set_multi_master_enabled(&mut self, enabled : bool) -> Result<()> {
    i2c::write_bit(self.dev_address, RA_I2C_MST_CTRL, MULT_MST_EN_BIT, enabled as u8)
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
    i2c::read_bit(self.dev_address, RA_I2C_MST_CTRL, WAIT_FOR_ES_BIT)
  }
  /** Set wait-for-external-sensor-data enabled value.
   * @param enabled New wait-for-external-sensor-data enabled value
   * @see getWaitForExternalSensorEnabled()
   * @see MPU9250_RA_I2C_MST_CTRL
   */
  pub fn set_wait_for_external_sensor_enabled(&mut self, enabled : bool) -> Result<()> {
    i2c::write_bit(self.dev_address, RA_I2C_MST_CTRL, WAIT_FOR_ES_BIT, enabled as u8)
  }
  /** Get Slave 3 FIFO enabled value.
   * When set to 1, this bit enables EXT_SENS_DATA registers (Registers 73 to 96)
   * associated with Slave 3 to be written into the FIFO buffer.
   * @return Current Slave 3 FIFO enabled value
   * @see MPU9250_RA_MST_CTRL
   */
  pub fn get_slave3_fifo_enabled(&mut self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_I2C_MST_CTRL, SLV_3_FIFO_EN_BIT)
  }
  /** Set Slave 3 FIFO enabled value.
   * @param enabled New Slave 3 FIFO enabled value
   * @see getSlave3FIFOEnabled()
   * @see MPU9250_RA_MST_CTRL
   */
  pub fn set_slave3_fifo_enabled(&mut self, enabled : bool) -> Result<()> {
    i2c::write_bit(self.dev_address, RA_I2C_MST_CTRL, SLV_3_FIFO_EN_BIT, enabled as u8)
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
    i2c::read_bit(self.dev_address, RA_I2C_MST_CTRL, I2C_MST_P_NSR_BIT)
  }
  /** Set slave read/write transition enabled value.
   * @param enabled New slave read/write transition enabled value
   * @see getSlaveReadWriteTransitionEnabled()
   * @see MPU9250_RA_I2C_MST_CTRL
   */
  pub fn set_slave_read_write_transition_enabled(&mut self, enabled : bool) -> Result<()> {
    i2c::write_bit(self.dev_address, RA_I2C_MST_CTRL, I2C_MST_P_NSR_BIT, enabled as u8)
  }

  /** Get I2C master clock speed.
   * I2C_MST_CLK is a 4 bit unsigned value which configures a divider on the
   * MPU-60X0 internal 8MHz clock. It sets the I2C master clock speed according to
   * the following table:
   *
   * <pre>
   * I2C_MST_CLK | I2C Master Clock Speed | 8MHz Clock Divider
   * ------------+------------------------+-------------------
   * 0           | 348kHz                 | 23
   * 1           | 333kHz                 | 24
   * 2           | 320kHz                 | 25
   * 3           | 308kHz                 | 26
   * 4           | 296kHz                 | 27
   * 5           | 286kHz                 | 28
   * 6           | 276kHz                 | 29
   * 7           | 267kHz                 | 30
   * 8           | 258kHz                 | 31
   * 9           | 500kHz                 | 16
   * 10          | 471kHz                 | 17
   * 11          | 444kHz                 | 18
   * 12          | 421kHz                 | 19
   * 13          | 400kHz                 | 20
   * 14          | 381kHz                 | 21
   * 15          | 364kHz                 | 22
   * </pre>
   *
   * @return Current I2C master clock speed
   * @see MPU9250_RA_I2C_MST_CTRL
   */
  pub fn get_master_clock_speed(&mut self) -> Result<u8> {
    i2c::read_bits(self.dev_address, RA_I2C_MST_CTRL, I2C_MST_CLK_BIT, I2C_MST_CLK_LENGTH)
  }
  /** Set I2C master clock speed.
   * @reparam speed Current I2C master clock speed
   * @see MPU9250_RA_I2C_MST_CTRL
   */
  pub fn set_master_clock_speed(&mut self, speed : u8) -> Result<()> {
    i2c::write_bits(self.dev_address, RA_I2C_MST_CTRL, I2C_MST_CLK_BIT, I2C_MST_CLK_LENGTH, speed)
  }
  
  // I2C_SLV* registers (Slave 0-3)
  
  /** Get the I2C address of the specified slave (0-3).
   * Note that Bit 7 (MSB) controls read/write mode. If Bit 7 is set, it's a read
   * operation, and if it is cleared, then it's a write operation. The remaining
   * bits (6-0) are the 7-bit device address of the slave device.
   *
   * In read mode, the result of the read is placed in the lowest available 
   * EXT_SENS_DATA register. For further information regarding the allocation of
   * read results, please refer to the EXT_SENS_DATA register description
   * (Registers 73 - 96).
   *
   * The MPU-9250 supports a total of five slaves, but Slave 4 has unique
   * characteristics, and so it has its own functions (getSlave4* and setSlave4*).
   *
   * I2C data transactions are performed at the Sample Rate, as defined in
   * Register 25. The user is responsible for ensuring that I2C data transactions
   * to and from each enabled Slave can be completed within a single period of the
   * Sample Rate.
   *
   * The I2C slave access rate can be reduced relative to the Sample Rate. This
   * reduced access rate is determined by I2C_MST_DLY (Register 52). Whether a
   * slave's access rate is reduced relative to the Sample Rate is determined by
   * I2C_MST_DELAY_CTRL (Register 103).
   *
   * The processing order for the slaves is fixed. The sequence followed for
   * processing the slaves is Slave 0, Slave 1, Slave 2, Slave 3 and Slave 4. If a
   * particular Slave is disabled it will be skipped.
   *
   * Each slave can either be accessed at the sample rate or at a reduced sample
   * rate. In a case where some slaves are accessed at the Sample Rate and some
   * slaves are accessed at the reduced rate, the sequence of accessing the slaves
   * (Slave 0 to Slave 4) is still followed. However, the reduced rate slaves will
   * be skipped if their access rate dictates that they should not be accessed
   * during that particular cycle. For further information regarding the reduced
   * access rate, please refer to Register 52. Whether a slave is accessed at the
   * Sample Rate or at the reduced rate is determined by the Delay Enable bits in
   * Register 103.
   *
   * @param num Slave number (0-3)
   * @return Current address for specified slave
   * @see MPU9250_RA_I2C_SLV0_ADDR
   */
  pub fn get_slave_address(&mut self, num : u8) -> Result<u8> {
    if (num > 3) {
      return Err(anyhow!("Slave number must be less than 4"));
    }
    i2c::read_byte(self.dev_address, RA_I2C_SLV0_ADDR + num*3)
  }
  /** Set the I2C address of the specified slave (0-3).
   * @param num Slave number (0-3)
   * @param address New address for specified slave
   * @see getSlaveAddress()
   * @see MPU9250_RA_I2C_SLV0_ADDR
   */
  pub fn set_slave_address(&mut self, num : u8, address : u8) -> Result<()> {
    if (num > 3) {
      return Err(anyhow!("Slave number must be less than 4"));
    }
    i2c::write_byte(self.dev_address, RA_I2C_SLV0_ADDR + num*3, address)
  }
  /** Get the active internal register for the specified slave (0-3).
   * Read/write operations for this slave will be done to whatever internal
   * register address is stored in this MPU register.
   *
   * The MPU-9250 supports a total of five slaves, but Slave 4 has unique
   * characteristics, and so it has its own functions.
   *
   * @param num Slave number (0-3)
   * @return Current active register for specified slave
   * @see MPU9250_RA_I2C_SLV0_REG
   */
  pub fn get_slave_register(&mut self, num : u8) -> Result<u8> {
    if (num > 3) {
      return Err(anyhow!("Slave number must be less than 4"));
    }
    i2c::read_byte(self.dev_address, RA_I2C_SLV0_REG + num*3)
  }
  /** Set the active internal register for the specified slave (0-3).
   * @param num Slave number (0-3)
   * @param register New active register for specified slave
   * @see getSlaveRegister()
   * @see MPU9250_RA_I2C_SLV0_REG
   */
  pub fn set_slave_register(&mut self, num : u8, register : u8) -> Result<()> {
    if (num > 3) {
      return Err(anyhow!("Slave number must be less than 4"));
    }
    i2c::write_byte(self.dev_address, RA_I2C_SLV0_REG + num*3, register)
  }

  /** Get whether the specified slave (0-3) is enabled.
  * @param num Slave number (0-3)
  * @return True if enabled, false otherwise
  * @see MPU9250_RA_I2C_SLV0_CTRL
  */
 pub fn get_slave_enabled(&self, num: u8) -> Result<u8> {
     if num > 3 {
         return Err(anyhow!("Slave number must be between 0 and 3"));
     }
     i2c::read_bit(self.dev_address, RA_I2C_SLV0_CTRL + num * 3, I2C_SLV_EN_BIT)
 }
 
 /** Set whether the specified slave (0-3) is enabled.
  * @param num Slave number (0-3)
  * @param enabled True to enable, false to disable
  * @see MPU9250_RA_I2C_SLV0_CTRL
  */
 pub fn set_slave_enabled(&self, num: u8, enabled: bool) -> Result<()> {
     if num > 3 {
         return Err(anyhow!("Slave number must be between 0 and 3"));
     }
     i2c::write_bit(self.dev_address, RA_I2C_SLV0_CTRL + num * 3, I2C_SLV_EN_BIT, enabled as u8)
 }

 /** Get word pair byte-swapping enabled for the specified slave (0-3).
  * When set to 1, this bit enables byte swapping. When byte swapping is enabled,
  * the high and low bytes of a word pair are swapped. Please refer to
  * I2C_SLV0_GRP for the pairing convention of the word pairs. When cleared to 0,
  * bytes transferred to and from Slave 0 will be written to EXT_SENS_DATA
  * registers in the order they were transferred.
  *
  * @param num Slave number (0-3)
  * @return Current word pair byte-swapping enabled value for specified slave
  * @see MPU9250_RA_I2C_SLV0_CTRL
  */
 pub fn get_slave_word_byte_swap(&self, num: u8) -> Result<u8> {
     if num > 3 {
         return Err(anyhow!("Slave number must be between 0 and 3"));
     }
     i2c::read_bit(self.dev_address, RA_I2C_SLV0_CTRL + num * 3, I2C_SLV_BYTE_SW_BIT)
 }
 
 /** Set word pair byte-swapping enabled for the specified slave (0-3).
  * @param num Slave number (0-3)
  * @param enabled New word pair byte-swapping enabled value for specified slave
  * @see getSlaveWordByteSwap()
  * @see MPU9250_RA_I2C_SLV0_CTRL
  */
 pub fn set_slave_word_byte_swap(&self, num: u8, enabled: bool) -> Result<()> {
     if num > 3 {
         return Err(anyhow!("Slave number must be between 0 and 3"));
     }
     i2c::write_bit(self.dev_address, RA_I2C_SLV0_CTRL + num * 3, I2C_SLV_BYTE_SW_BIT, enabled as u8)
 }

/** Get write mode for the specified slave (0-3).
 * When set to 1, the transaction will read or write data only. When cleared to
 * 0, the transaction will write a register address prior to reading or writing
 * data. This should equal 0 when specifying the register address within the
 * Slave device to/from which the ensuing data transaction will take place.
 *
 * @param num Slave number (0-3)
 * @return Current write mode for specified slave (0 = register address + data, 1 = data only)
 * @see MPU9250_RA_I2C_SLV0_CTRL
 */
pub fn get_slave_write_mode(&self, num: u8) -> Result<u8> {
    if num > 3 {
        return Err(anyhow!("Slave number must be between 0 and 3"));
    }
    i2c::read_bit(self.dev_address, RA_I2C_SLV0_CTRL + num * 3, I2C_SLV_REG_DIS_BIT)
}

/** Set write mode for the specified slave (0-3).
 * @param num Slave number (0-3)
 * @param mode New write mode for specified slave (0 = register address + data, 1 = data only)
 * @see getSlaveWriteMode()
 * @see MPU9250_RA_I2C_SLV0_CTRL
 */
pub fn set_slave_write_mode(&self, num: u8, mode: bool) -> Result<()> {
    if num > 3 {
        return Err(anyhow!("Slave number must be between 0 and 3"));
    }
    i2c::write_bit(self.dev_address, RA_I2C_SLV0_CTRL + num * 3, I2C_SLV_REG_DIS_BIT, mode as u8)
}

/** Get word pair grouping order offset for the specified slave (0-3).
 * This specifies the grouping order of word pairs received from registers.
 * When cleared to 0, bytes from register addresses 0 and 1, 2 and 3, etc (even,
 * then odd register addresses) are paired to form a word. When set to 1, bytes
 * from register addresses are paired 1 and 2, 3 and 4, etc. (odd, then even
 * register addresses) are paired to form a word.
 *
 * @param num Slave number (0-3)
 * @return Current word pair grouping order offset for specified slave
 * @see MPU9250_RA_I2C_SLV0_CTRL
 */
pub fn get_slave_word_group_offset(&self, num: u8) -> Result<u8> {
    if num > 3 {
        return Err(anyhow!("Slave number must be between 0 and 3"));
    }
    i2c::read_bit(self.dev_address, RA_I2C_SLV0_CTRL + num * 3, I2C_SLV_GRP_BIT)
}

/** Set word pair grouping order offset for the specified slave (0-3).
 * @param num Slave number (0-3)
 * @param enabled New word pair grouping order offset for specified slave
 * @see getSlaveWordGroupOffset()
 * @see MPU9250_RA_I2C_SLV0_CTRL
 */
pub fn set_slave_word_group_offset(&self, num: u8, enabled: bool) -> Result<()> {
    if num > 3 {
        return Err(anyhow!("Slave number must be between 0 and 3"));
    }
    i2c::write_bit(self.dev_address, RA_I2C_SLV0_CTRL + num * 3, I2C_SLV_GRP_BIT, enabled as u8)
}

/** Get number of bytes to read for the specified slave (0-3).
 * Specifies the number of bytes transferred to and from Slave 0. Clearing this
 * bit to 0 is equivalent to disabling the register by writing 0 to I2C_SLV0_EN.
 * @param num Slave number (0-3)
 * @return Number of bytes to read for specified slave
 * @see MPU9250_RA_I2C_SLV0_CTRL
 */
pub fn get_slave_data_length(&self, num: u8) -> Result<u8> {
    if num > 3 {
        return Err(anyhow!("Slave number must be between 0 and 3"));
    }
    i2c::read_bits(self.dev_address, RA_I2C_SLV0_CTRL + num * 3, I2C_SLV_LEN_BIT, I2C_SLV_LEN_LENGTH)
}

/** Set number of bytes to read for the specified slave (0-3).
 * @param num Slave number (0-3)
 * @param length Number of bytes to read for specified slave
 * @see getSlaveDataLength()
 * @see MPU9250_RA_I2C_SLV0_CTRL
 */
pub fn set_slave_data_length(&self, num: u8, length: u8) -> Result<()> {
    if num > 3 {
        return Err(anyhow!("Slave number must be between 0 and 3"));
    }
    i2c::write_bits(self.dev_address, RA_I2C_SLV0_CTRL + num * 3, I2C_SLV_LEN_BIT, I2C_SLV_LEN_LENGTH, length)
}

/** Get the I2C address of Slave 4.
 * Note that Bit 7 (MSB) controls read/write mode. If Bit 7 is set, it's a read
 * operation, and if it is cleared, then it's a write operation. The remaining
 * bits (6-0) are the 7-bit device address of the slave device.
 *
 * @return Current address for Slave 4
 * @see getSlaveAddress()
 * @see MPU9250_RA_I2C_SLV4_ADDR
 */
pub fn get_slave4_address(&self) -> Result<u8> {
    i2c::read_byte(self.dev_address, RA_I2C_SLV4_ADDR)
}

/** Set the I2C address of Slave 4.
 * @param address New address for Slave 4
 * @see getSlave4Address()
 * @see MPU9250_RA_I2C_SLV4_ADDR
 */
pub fn set_slave4_address(&self, address: u8) -> Result<()> {
    i2c::write_byte(self.dev_address, RA_I2C_SLV4_ADDR, address)
}

/** Get the active internal register for the Slave 4.
 * Read/write operations for this slave will be done to whatever internal
 * register address is stored in this MPU register.
 *
 * @return Current active register for Slave 4
 * @see MPU9250_RA_I2C_SLV4_REG
 */
pub fn get_slave4_register(&self) -> Result<u8> {
    i2c::read_byte(self.dev_address, RA_I2C_SLV4_REG)
}

/** Set the active internal register for Slave 4.
 * @param reg New active register for Slave 4
 * @see getSlave4Register()
 * @see MPU9250_RA_I2C_SLV4_REG
 */
pub fn set_slave4_register(&self, reg: u8) -> Result<()> {
    i2c::write_byte(self.dev_address, RA_I2C_SLV4_REG, reg)
}

/** Set new byte to write to Slave 4.
 * This register stores the data to be written into the Slave 4. If I2C_SLV4_RW
 * is set 1 (set to read), this register has no effect.
 * @param data New byte to write to Slave 4
 * @see MPU9250_RA_I2C_SLV4_DO
 */
pub fn set_slave4_output_byte(&self, data: u8) -> Result<()> {
    i2c::write_byte(self.dev_address, RA_I2C_SLV4_DO, data)
}

/** Get the enabled value for the Slave 4.
 * When set to 1, this bit enables Slave 4 for data transfer operations. When
 * cleared to 0, this bit disables Slave 4 from data transfer operations.
 * @return Current enabled value for Slave 4
 * @see MPU9250_RA_I2C_SLV4_CTRL
 */
pub fn get_slave4_enabled(&self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_I2C_SLV4_CTRL, I2C_SLV4_EN_BIT)
}

/** Set the enabled value for Slave 4.
 * @param enabled New enabled value for Slave 4
 * @see getSlave4Enabled()
 * @see MPU9250_RA_I2C_SLV4_CTRL
 */
pub fn set_slave4_enabled(&self, enabled: bool) -> Result<()> {
    i2c::write_bit(self.dev_address, RA_I2C_SLV4_CTRL, I2C_SLV4_EN_BIT, enabled as u8)
}

/** Get the enabled value for Slave 4 transaction interrupts.
 * When set to 1, this bit enables the generation of an interrupt signal upon
 * completion of a Slave 4 transaction. When cleared to 0, this bit disables the
 * generation of an interrupt signal upon completion of a Slave 4 transaction.
 * The interrupt status can be observed in Register 54.
 *
 * @return Current enabled value for Slave 4 transaction interrupts.
 * @see MPU9250_RA_I2C_SLV4_CTRL
 */
pub fn get_slave4_interrupt_enabled(&self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_I2C_SLV4_CTRL, I2C_SLV4_INT_EN_BIT)
}

/** Set the enabled value for Slave 4 transaction interrupts.
 * @param enabled New enabled value for Slave 4 transaction interrupts.
 * @see getSlave4InterruptEnabled()
 * @see MPU9250_RA_I2C_SLV4_CTRL
 */
pub fn set_slave4_interrupt_enabled(&self, enabled: bool) -> Result<()> {
    i2c::write_bit(self.dev_address, RA_I2C_SLV4_CTRL, I2C_SLV4_INT_EN_BIT, enabled as u8)
}

/** Get write mode for Slave 4.
 * When set to 1, the transaction will read or write data only. When cleared to
 * 0, the transaction will write a register address prior to reading or writing
 * data. This should equal 0 when specifying the register address within the
 * Slave device to/from which the ensuing data transaction will take place.
 *
 * @return Current write mode for Slave 4 (0 = register address + data, 1 = data only)
 * @see MPU9250_RA_I2C_SLV4_CTRL
 */
pub fn get_slave4_write_mode(&self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_I2C_SLV4_CTRL, I2C_SLV4_REG_DIS_BIT)
}

/** Set write mode for the Slave 4.
 * @param mode New write mode for Slave 4 (0 = register address + data, 1 = data only)
 * @see getSlave4WriteMode()
 * @see MPU9250_RA_I2C_SLV4_CTRL
 */
pub fn set_slave4_write_mode(&self, mode: bool) -> Result<()> {
    i2c::write_bit(self.dev_address, RA_I2C_SLV4_CTRL, I2C_SLV4_REG_DIS_BIT, mode as u8)
}

/** Get Slave 4 master delay value.
 * This configures the reduced access rate of I2C slaves relative to the Sample
 * Rate. When a slave's access rate is decreased relative to the Sample Rate,
 * the slave is accessed every: 
 *
 *  1 / (1 + I2C_MST_DLY) // samples
 *
 * This base Sample Rate in turn is determined by SMPLRT_DIV (register 25) and
 * DLPF_CFG (register 26). Whether a slave's access rate is reduced relative to
 * the Sample Rate is determined by I2C_MST_DELAY_CTRL (register 103). For
 * further information regarding the Sample Rate, please refer to register 25.
 *
 * @return Current Slave 4 master delay value
 * @see MPU9250_RA_I2C_SLV4_CTRL
 */
pub fn get_slave4_master_delay(&self) -> Result<u8> {
    i2c::read_bits(self.dev_address, RA_I2C_SLV4_CTRL, I2C_SLV4_MST_DLY_BIT, I2C_SLV4_MST_DLY_LENGTH)
}

/** Set Slave 4 master delay value.
 * @param delay New Slave 4 master delay value
 * @see getSlave4MasterDelay()
 * @see MPU9250_RA_I2C_SLV4_CTRL
 */
pub fn set_slave4_master_delay(&self, delay: u8) -> Result<()> {
    i2c::write_bits(self.dev_address, RA_I2C_SLV4_CTRL, I2C_SLV4_MST_DLY_BIT, I2C_SLV4_MST_DLY_LENGTH, delay)
}

/** Get last available byte read from Slave 4.
 * This register stores the data read from Slave 4. This field is populated
 * after a read transaction.
 * @return Last available byte read from to Slave 4
 * @see MPU9250_RA_I2C_SLV4_DI
 */
pub fn get_slave4_input_byte(&self) -> Result<u8> {
    i2c::read_byte(self.dev_address, RA_I2C_SLV4_DI)
}

// I2C_MST_STATUS register

/// Get FSYNC interrupt status.
/// This bit reflects the status of the FSYNC interrupt from an external device
/// into the MPU-60X0. This is used as a way to pass an external interrupt
/// through the MPU-60X0 to the host application processor. When set to 1, this
/// bit will cause an interrupt if FSYNC_INT_EN is asserted in INT_PIN_CFG
/// (Register 55).
/// @return FSYNC interrupt status
/// @see MPU9250_RA_I2C_MST_STATUS
pub fn get_passthrough_status(&mut self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_I2C_MST_STATUS, MST_PASS_THROUGH_BIT)
}

/// Get Slave 4 transaction done status.
/// Automatically sets to 1 when a Slave 4 transaction has completed. This
/// triggers an interrupt if the I2C_MST_INT_EN bit in the INT_ENABLE register
/// (Register 56) is asserted and if the SLV_4_DONE_INT bit is asserted in the
/// I2C_SLV4_CTRL register (Register 52).
/// @return Slave 4 transaction done status
/// @see MPU9250_RA_I2C_MST_STATUS
pub fn get_slave4_is_done(&mut self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_I2C_MST_STATUS, MST_I2C_SLV4_DONE_BIT)
}

/// Get master arbitration lost status.
/// This bit automatically sets to 1 when the I2C Master has lost arbitration of
/// the auxiliary I2C bus (an error condition). This triggers an interrupt if the
/// I2C_MST_INT_EN bit in the INT_ENABLE register (Register 56) is asserted.
/// @return Master arbitration lost status
/// @see MPU9250_RA_I2C_MST_STATUS
pub fn get_lost_arbitration(&mut self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_I2C_MST_STATUS, MST_I2C_LOST_ARB_BIT)
}

/// Get Slave 4 NACK status.
/// This bit automatically sets to 1 when the I2C Master receives a NACK in a
/// transaction with Slave 4. This triggers an interrupt if the I2C_MST_INT_EN
/// bit in the INT_ENABLE register (Register 56) is asserted.
/// @return Slave 4 NACK interrupt status
/// @see MPU9250_RA_I2C_MST_STATUS
pub fn get_slave4_nack(&mut self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_I2C_MST_STATUS, MST_I2C_SLV4_NACK_BIT)
}

/// Get Slave 3 NACK status.
/// This bit automatically sets to 1 when the I2C Master receives a NACK in a
/// transaction with Slave 3. This triggers an interrupt if the I2C_MST_INT_EN
/// bit in the INT_ENABLE register (Register 56) is asserted.
/// @return Slave 3 NACK interrupt status
/// @see MPU9250_RA_I2C_MST_STATUS
pub fn get_slave3_nack(&mut self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_I2C_MST_STATUS, MST_I2C_SLV3_NACK_BIT)
}

/// Get Slave 2 NACK status.
/// This bit automatically sets to 1 when the I2C Master receives a NACK in a
/// transaction with Slave 2. This triggers an interrupt if the I2C_MST_INT_EN
/// bit in the INT_ENABLE register (Register 56) is asserted.
/// @return Slave 2 NACK interrupt status
/// @see MPU9250_RA_I2C_MST_STATUS
pub fn get_slave2_nack(&mut self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_I2C_MST_STATUS, MST_I2C_SLV2_NACK_BIT)
}

/// Get Slave 1 NACK status.
/// This bit automatically sets to 1 when the I2C Master receives a NACK in a
/// transaction with Slave 1. This triggers an interrupt if the I2C_MST_INT_EN
/// bit in the INT_ENABLE register (Register 56) is asserted.
/// @return Slave 1 NACK interrupt status
/// @see MPU9250_RA_I2C_MST_STATUS
pub fn get_slave1_nack(&mut self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_I2C_MST_STATUS, MST_I2C_SLV1_NACK_BIT)
}

/// Get Slave 0 NACK status.
/// This bit automatically sets to 1 when the I2C Master receives a NACK in a
/// transaction with Slave 0. This triggers an interrupt if the I2C_MST_INT_EN
/// bit in the INT_ENABLE register (Register 56) is asserted.
/// @return Slave 0 NACK interrupt status
/// @see MPU9250_RA_I2C_MST_STATUS
pub fn get_slave0_nack(&mut self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_I2C_MST_STATUS, MST_I2C_SLV0_NACK_BIT)
}

// INT_PIN_CFG register

/// Get interrupt logic level mode.
/// Will be set 0 for active-high, 1 for active-low.
/// @return Current interrupt mode (0=active-high, 1=active-low)
/// @see MPU9250_RA_INT_PIN_CFG
/// @see MPU9250_INTCFG_INT_LEVEL_BIT
pub fn get_interrupt_mode(&mut self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_INT_PIN_CFG, INTCFG_INT_LEVEL_BIT)
}

/// Set interrupt logic level mode.
/// @param mode New interrupt mode (0=active-high, 1=active-low)
/// @see get_interrupt_mode()
/// @see MPU9250_RA_INT_PIN_CFG
/// @see MPU9250_INTCFG_INT_LEVEL_BIT
pub fn set_interrupt_mode(&mut self, mode: u8) -> Result<()> {
    i2c::write_bit(self.dev_address, RA_INT_PIN_CFG, INTCFG_INT_LEVEL_BIT, mode)
}

/// Get interrupt drive mode.
/// Will be set 0 for push-pull, 1 for open-drain.
/// @return Current interrupt drive mode (0=push-pull, 1=open-drain)
/// @see MPU9250_RA_INT_PIN_CFG
/// @see MPU9250_INTCFG_INT_OPEN_BIT
pub fn get_interrupt_drive(&mut self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_INT_PIN_CFG, INTCFG_INT_OPEN_BIT)
}

/// Set interrupt drive mode.
/// @param drive New interrupt drive mode (0=push-pull, 1=open-drain)
/// @see get_interrupt_drive()
/// @see MPU9250_RA_INT_PIN_CFG
/// @see MPU9250_INTCFG_INT_OPEN_BIT
pub fn set_interrupt_drive(&mut self, drive: bool) -> Result<()> {
    i2c::write_bit(self.dev_address, RA_INT_PIN_CFG, INTCFG_INT_OPEN_BIT, drive as u8)
}

/// Get interrupt latch mode.
/// Will be set 0 for 50us-pulse, 1 for latch-until-int-cleared.
/// @return Current latch mode (0=50us-pulse, 1=latch-until-int-cleared)
/// @see MPU9250_RA_INT_PIN_CFG
/// @see MPU9250_INTCFG_LATCH_INT_EN_BIT
pub fn get_interrupt_latch(&mut self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_INT_PIN_CFG, INTCFG_LATCH_INT_EN_BIT)
}

/// Set interrupt latch mode.
/// @param latch New latch mode (0=50us-pulse, 1=latch-until-int-cleared)
/// @see get_interrupt_latch()
/// @see MPU9250_RA_INT_PIN_CFG
/// @see MPU9250_INTCFG_LATCH_INT_EN_BIT
pub fn set_interrupt_latch(&mut self, latch: u8) -> Result<()> {
    i2c::write_bit(self.dev_address, RA_INT_PIN_CFG, INTCFG_LATCH_INT_EN_BIT, latch)
}

/// Get interrupt latch clear mode.
/// Will be set 0 for status-read-only, 1 for any-register-read.
/// @return Current latch clear mode (0=status-read-only, 1=any-register-read)
/// @see MPU9250_RA_INT_PIN_CFG
/// @see MPU9250_INTCFG_INT_RD_CLEAR_BIT
pub fn get_interrupt_latch_clear(&mut self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_INT_PIN_CFG, INTCFG_INT_RD_CLEAR_BIT)
}

/// Set interrupt latch clear mode.
/// @param clear New latch clear mode (0=status-read-only, 1=any-register-read)
/// @see get_interrupt_latch_clear()
/// @see MPU9250_RA_INT_PIN_CFG
/// @see MPU9250_INTCFG_INT_RD_CLEAR_BIT
pub fn set_interrupt_latch_clear(&mut self, clear: bool) -> Result<()> {
    return i2c::write_bit(self.dev_address, RA_INT_PIN_CFG, INTCFG_INT_RD_CLEAR_BIT, clear as u8)
}

/// Get FSYNC interrupt logic level mode.
/// @return Current FSYNC interrupt mode (0=active-high, 1=active-low)
/// @see get_fsync_interrupt_mode()
/// @see MPU9250_RA_INT_PIN_CFG
/// @see MPU9250_INTCFG_FSYNC_INT_LEVEL_BIT
pub fn get_fsync_interrupt_level(&mut self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_INT_PIN_CFG, INTCFG_FSYNC_INT_LEVEL_BIT)
}

/// Set FSYNC interrupt logic level mode.
/// @param level New FSYNC interrupt mode (0=active-high, 1=active-low)
/// @see get_fsync_interrupt_mode()
/// @see MPU9250_RA_INT_PIN_CFG
/// @see MPU9250_INTCFG_FSYNC_INT_LEVEL_BIT
pub fn set_fsync_interrupt_level(&mut self, level: u8) -> Result<()> {
    i2c::write_bit(self.dev_address, RA_INT_PIN_CFG, INTCFG_FSYNC_INT_LEVEL_BIT, level)
}

/// Get FSYNC pin interrupt enabled setting.
/// Will be set 0 for disabled, 1 for enabled.
/// @return Current interrupt enabled setting
/// @see MPU9250_RA_INT_PIN_CFG
/// @see MPU9250_INTCFG_FSYNC_INT_EN_BIT
pub fn get_fsync_interrupt_enabled(&mut self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_INT_PIN_CFG, INTCFG_FSYNC_INT_EN_BIT)
}

/// Set FSYNC pin interrupt enabled setting.
/// @param enabled New FSYNC pin interrupt enabled setting
/// @see get_fsync_interrupt_enabled()
/// @see MPU9250_RA_INT_PIN_CFG
/// @see MPU9250_INTCFG_FSYNC_INT_EN_BIT
pub fn set_fsync_interrupt_enabled(&mut self, enabled: bool) -> Result<()> {
    i2c::write_bit(self.dev_address, RA_INT_PIN_CFG, INTCFG_FSYNC_INT_EN_BIT, enabled as u8)
}

/// Get I2C bypass enabled status.
/// When this bit is equal to 1 and I2C_MST_EN (Register 106 bit[5]) is equal to
/// 0, the host application processor will be able to directly access the
/// auxiliary I2C bus of the MPU-60X0. When this bit is equal to 0, the host
/// application processor will not be able to directly access the auxiliary I2C
/// bus of the MPU-60X0 regardless of the state of I2C_MST_EN (Register 106
/// bit[5]).
/// @return Current I2C bypass enabled status
/// @see MPU9250_RA_INT_PIN_CFG
/// @see MPU9250_INTCFG_I2C_BYPASS_EN_BIT
pub fn get_i2c_bypass_enabled(&mut self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_INT_PIN_CFG, INTCFG_I2C_BYPASS_EN_BIT)
}

/// Set I2C bypass enabled status.
/// When this bit is equal to 1 and I2C_MST_EN (Register 106 bit[5]) is equal to
/// 0, the host application processor will be able to directly access the
/// auxiliary I2C bus of the MPU-60X0. When this bit is equal to 0, the host
/// application processor will not be able to directly access the auxiliary I2C
/// bus of the MPU-60X0 regardless of the state of I2C_MST_EN (Register 106
/// bit[5]).
/// @param enabled New I2C bypass enabled status
/// @see get_i2c_bypass_enabled()
/// @see MPU9250_RA_INT_PIN_CFG
/// @see MPU9250_INTCFG_I2C_BYPASS_EN_BIT
pub fn set_i2c_bypass_enabled(&mut self, enabled: bool) -> Result<()> {
    i2c::write_bit(self.dev_address, RA_INT_PIN_CFG, INTCFG_I2C_BYPASS_EN_BIT, enabled as u8)
}

/// Get reference clock output enabled status.
/// When this bit is equal to 1, a reference clock output is provided at the
/// CLKOUT pin. When this bit is equal to 0, the clock output is disabled. For
/// further information regarding CLKOUT, please refer to the MPU-60X0 Product
/// Specification document.
/// @return Current reference clock output enabled status
/// @see MPU9250_RA_INT_PIN_CFG
/// @see MPU9250_INTCFG_CLKOUT_EN_BIT
pub fn get_clock_output_enabled(&mut self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_INT_PIN_CFG, INTCFG_CLKOUT_EN_BIT)
}

/// Set reference clock output enabled status.
/// When this bit is equal to 1, a reference clock output is provided at the
/// CLKOUT pin. When this bit is equal to 0, the clock output is disabled. For
/// further information regarding CLKOUT, please refer to the MPU-60X0 Product
/// Specification document.
/// @param enabled New reference clock output enabled status
/// @see get_clock_output_enabled()
/// @see MPU9250_RA_INT_PIN_CFG
/// @see MPU9250_INTCFG_CLKOUT_EN_BIT
pub fn set_clock_output_enabled(&mut self, enabled: bool) -> Result<()> {
    i2c::write_bit(self.dev_address, RA_INT_PIN_CFG, INTCFG_CLKOUT_EN_BIT, enabled as u8)
}

/// Get full interrupt enabled status.
/// Full register byte for all interrupts, for quick reading. Each bit will be
/// set 0 for disabled, 1 for enabled.
/// @return Current interrupt enabled status
/// @see MPU9250_RA_INT_ENABLE
/// @see MPU9250_INTERRUPT_FF_BIT
pub fn get_int_enabled(&mut self) -> Result<u8> {
    i2c::read_byte(self.dev_address, RA_INT_ENABLE)
}

/// Set full interrupt enabled status.
/// Full register byte for all interrupts, for quick reading. Each bit should be
/// set 0 for disabled, 1 for enabled.
/// @param enabled New interrupt enabled status
/// @see get_int_freefall_enabled()
/// @see MPU9250_RA_INT_ENABLE
/// @see MPU9250_INTERRUPT_FF_BIT
pub fn set_int_enabled(&mut self, enabled: bool) -> Result<()> {
    i2c::write_byte(self.dev_address, RA_INT_ENABLE, enabled as u8)
}

/// Get Free Fall interrupt enabled status.
/// Will be set 0 for disabled, 1 for enabled.
/// @return Current interrupt enabled status
/// @see MPU9250_RA_INT_ENABLE
/// @see MPU9250_INTERRUPT_FF_BIT
pub fn get_int_freefall_enabled(&mut self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_INT_ENABLE, INTERRUPT_FF_BIT)
}

/// Set Free Fall interrupt enabled status.
/// @param enabled New interrupt enabled status
/// @see get_int_freefall_enabled()
/// @see MPU9250_RA_INT_ENABLE
/// @see MPU9250_INTERRUPT_FF_BIT
pub fn set_int_freefall_enabled(&mut self, enabled: bool) -> Result<()> {
    i2c::write_bit(self.dev_address, RA_INT_ENABLE, INTERRUPT_FF_BIT, enabled as u8)
}

/// Get Motion Detection interrupt enabled status.
/// Will be set 0 for disabled, 1 for enabled.
/// @return Current interrupt enabled status
/// @see MPU9250_RA_INT_ENABLE
/// @see MPU9250_INTERRUPT_MOT_BIT
pub fn get_int_motion_enabled(&mut self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_INT_ENABLE, INTERRUPT_MOT_BIT)
}

/// Set Motion Detection interrupt enabled status.
/// @param enabled New interrupt enabled status
/// @see get_int_motion_enabled()
/// @see MPU9250_RA_INT_ENABLE
/// @see MPU9250_INTERRUPT_MOT_BIT
pub fn set_int_motion_enabled(&mut self, enabled: bool) -> Result<()> {
    i2c::write_bit(self.dev_address, RA_INT_ENABLE, INTERRUPT_MOT_BIT, enabled as u8)
}

/// Get Zero Motion Detection interrupt enabled status.
/// Will be set 0 for disabled, 1 for enabled.
/// @return Current interrupt enabled status
/// @see MPU9250_RA_INT_ENABLE
/// @see MPU9250_INTERRUPT_ZMOT_BIT
pub fn get_int_zero_motion_enabled(&mut self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_INT_ENABLE, INTERRUPT_ZMOT_BIT)
}

/// Set Zero Motion Detection interrupt enabled status.
/// @param enabled New interrupt enabled status
/// @see get_int_zero_motion_enabled()
/// @see MPU9250_RA_INT_ENABLE
/// @see MPU9250_INTERRUPT_ZMOT_BIT
pub fn set_int_zero_motion_enabled(&mut self, enabled: bool) -> Result<()> {
    i2c::write_bit(self.dev_address, RA_INT_ENABLE, INTERRUPT_ZMOT_BIT, enabled as u8)
}

/// Get FIFO Buffer Overflow interrupt enabled status.
/// Will be set 0 for disabled, 1 for enabled.
/// @return Current interrupt enabled status
/// @see MPU9250_RA_INT_ENABLE
/// @see MPU9250_INTERRUPT_FIFO_OFLOW_BIT
pub fn get_int_fifo_buffer_overflow_enabled(&mut self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_INT_ENABLE, INTERRUPT_FIFO_OFLOW_BIT)
}

/// Set FIFO Buffer Overflow interrupt enabled status.
/// @param enabled New interrupt enabled status
/// @see get_int_fifo_buffer_overflow_enabled()
/// @see MPU9250_RA_INT_ENABLE
/// @see MPU9250_INTERRUPT_FIFO_OFLOW_BIT
pub fn set_int_fifo_buffer_overflow_enabled(&mut self, enabled: bool) -> Result<()> {
    i2c::write_bit(self.dev_address, RA_INT_ENABLE, INTERRUPT_FIFO_OFLOW_BIT, enabled as u8)
}

/// Get I2C Master interrupt enabled status.
/// This enables any of the I2C Master interrupt sources to generate an
/// interrupt. Will be set 0 for disabled, 1 for enabled.
/// @return Current interrupt enabled status
/// @see MPU9250_RA_INT_ENABLE
/// @see MPU9250_INTERRUPT_I2C_MST_INT_BIT
pub fn get_int_i2c_master_enabled(&mut self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_INT_ENABLE, INTERRUPT_I2C_MST_INT_BIT)
}

/// Set I2C Master interrupt enabled status.
/// @param enabled New interrupt enabled status
/// @see get_int_i2c_master_enabled()
/// @see MPU9250_RA_INT_ENABLE
/// @see MPU9250_INTERRUPT_I2C_MST_INT_BIT
pub fn set_int_i2c_master_enabled(&mut self, enabled: bool) -> Result<()> {
    i2c::write_bit(self.dev_address, RA_INT_ENABLE, INTERRUPT_I2C_MST_INT_BIT, enabled as u8)
}

/// Get Data Ready interrupt enabled setting.
/// This event occurs each time a write operation to all of the sensor registers
/// has been completed. Will be set 0 for disabled, 1 for enabled.
/// @return Current interrupt enabled status
/// @see MPU9250_RA_INT_ENABLE
/// @see MPU9250_INTERRUPT_DATA_RDY_BIT
pub fn get_int_data_ready_enabled(&mut self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_INT_ENABLE, INTERRUPT_DATA_RDY_BIT)
}

/// Set Data Ready interrupt enabled status.
/// @param enabled New interrupt enabled status
/// @see get_int_data_ready_enabled()
/// @see MPU9250_RA_INT_CFG
/// @see MPU9250_INTERRUPT_DATA_RDY_BIT
pub fn set_int_data_ready_enabled(&mut self, enabled: bool) -> Result<()> {
    i2c::write_bit(self.dev_address, RA_INT_ENABLE, INTERRUPT_DATA_RDY_BIT, enabled as u8)
}

/// Get full set of interrupt status bits.
/// These bits clear to 0 after the register has been read. Very useful
/// for getting multiple INT statuses, since each single bit read clears
/// all of them because it has to read the whole byte.
/// @return Current interrupt status
/// @see MPU9250_RA_INT_STATUS
pub fn get_int_status(&mut self) -> Result<u8> {
    i2c::read_byte(self.dev_address, RA_INT_STATUS)
}

/// Get Free Fall interrupt status.
/// This bit automatically sets to 1 when a Free Fall interrupt has been
/// generated. The bit clears to 0 after the register has been read.
/// @return Current interrupt status
/// @see MPU9250_RA_INT_STATUS
/// @see MPU9250_INTERRUPT_FF_BIT
pub fn get_int_freefall_status(&mut self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_INT_STATUS, INTERRUPT_FF_BIT)
}

/// Get Motion Detection interrupt status.
/// This bit automatically sets to 1 when a Motion Detection interrupt has been
/// generated. The bit clears to 0 after the register has been read.
/// @return Current interrupt status
/// @see MPU9250_RA_INT_STATUS
/// @see MPU9250_INTERRUPT_MOT_BIT
pub fn get_int_motion_status(&mut self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_INT_STATUS, INTERRUPT_MOT_BIT)
}

/// Get Zero Motion Detection interrupt status.
/// This bit automatically sets to 1 when a Zero Motion Detection interrupt has
/// been generated. The bit clears to 0 after the register has been read.
/// @return Current interrupt status
/// @see MPU9250_RA_INT_STATUS
/// @see MPU9250_INTERRUPT_ZMOT_BIT
pub fn get_int_zero_motion_status(&mut self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_INT_STATUS, INTERRUPT_ZMOT_BIT)
}

/// Get FIFO Buffer Overflow interrupt status.
/// This bit automatically sets to 1 when a Free Fall interrupt has been
/// generated. The bit clears to 0 after the register has been read.
/// @return Current interrupt status
/// @see MPU9250_RA_INT_STATUS
/// @see MPU9250_INTERRUPT_FIFO_OFLOW_BIT
pub fn get_int_fifo_buffer_overflow_status(&mut self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_INT_STATUS, INTERRUPT_FIFO_OFLOW_BIT)
}

/// Get I2C Master interrupt status.
/// This bit automatically sets to 1 when an I2C Master interrupt has been
/// generated. For a list of I2C Master interrupts, please refer to Register 54.
/// The bit clears to 0 after the register has been read.
/// @return Current interrupt status
/// @see MPU9250_RA_INT_STATUS
/// @see MPU9250_INTERRUPT_I2C_MST_INT_BIT
pub fn get_int_i2c_master_status(&mut self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_INT_STATUS, INTERRUPT_I2C_MST_INT_BIT)
}

/// Get Data Ready interrupt status.
/// This bit automatically sets to 1 when a Data Ready interrupt has been
/// generated. The bit clears to 0 after the register has been read.
/// @return Current interrupt status
/// @see MPU9250_RA_INT_STATUS
/// @see MPU9250_INTERRUPT_DATA_RDY_BIT
pub fn get_int_data_ready_status(&mut self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_INT_STATUS, INTERRUPT_DATA_RDY_BIT)
}

/** Get raw 9-axis motion sensor readings (accel/gyro/compass).
 * FUNCTION NOT FULLY IMPLEMENTED YET.
 * @see getMotion6()
 * @see getAcceleration()
 * @see getRotation()
 * @see MPU9250_RA_ACCEL_XOUT_H
 */
pub fn get_motion_9(&mut self) -> Result<(AccelerometerData, GyroscopeData, MagnetometerData)> {
  //get accel and gyro
  let (accelerometer_data, gyroscope_data) = self.get_motion_6()?;
  // This must be repeated for each pass through read to the peripheral device 
  // This does not seem to be documented in the data sheet
  self.set_i2c_bypass_enabled(true);
  std::thread::sleep(std::time::Duration::from_millis(10));
  self.set_magnetometer_enabled(true);
  std::thread::sleep(std::time::Duration::from_millis(10));
  //read mag
  let mut buffer = [0; 6];
  i2c::read_bytes(mpu9150::RA_MAG_ADDRESS, mpu9150::RA_MAG_XOUT_L, 6, &mut buffer)?;
  let magnetometer_data = MagnetometerData {
      x : ((buffer[1] as i16) << 8) | buffer[0] as i16,
      y : ((buffer[3] as i16) << 8) | buffer[2] as i16,
      z : ((buffer[5] as i16) << 8) | buffer[4] as i16,
  };
  Ok((accelerometer_data, gyroscope_data, magnetometer_data))
}


  /** Get raw 6-axis motion sensor readings (accel/gyro).
   * Retrieves all currently available motion sensor values.
   * @see getAcceleration()
   * @see getRotation()
   * @see MPU9250_RA_ACCEL_XOUT_H
   */
pub fn get_motion_6(&mut self) -> Result<(AccelerometerData, GyroscopeData)> {
  let mut buffer = [0; 14];
  i2c::read_bytes(self.dev_address, RA_ACCEL_XOUT_H, 14, &mut buffer)?;
  let accelerometer_data = AccelerometerData {
      x : ((buffer[0] as i16) << 8) | buffer[1] as i16,
      y : ((buffer[2] as i16) << 8) | buffer[3] as i16,
      z : ((buffer[4] as i16) << 8) | buffer[5] as i16,
  };
  let gyroscope_data = GyroscopeData {
      x : ((buffer[8] as i16) << 8) | buffer[9] as i16,
      y : ((buffer[10] as i16) << 8) | buffer[11] as i16,
      z : ((buffer[12] as i16) << 8) | buffer[13] as i16,
  };
  Ok((accelerometer_data, gyroscope_data))
  
}


  /** Get 3-axis accelerometer readings.
   * These registers store the most recent accelerometer measurements.
   * Accelerometer measurements are written to these registers at the Sample Rate
   * as defined in Register 25.
   *
   * The accelerometer measurement registers, along with the temperature
   * measurement registers, gyroscope measurement registers, and external sensor
   * data registers, are composed of two sets of registers: an internal register
   * set and a user-facing read register set.
   *
   * The data within the accelerometer sensors' internal register set is always
   * updated at the Sample Rate. Meanwhile, the user-facing read register set
   * duplicates the internal register set's data values whenever the serial
   * interface is idle. This guarantees that a burst read of sensor registers will
   * read measurements from the same sampling instant. Note that if burst reads
   * are not used, the user is responsible for ensuring a set of single byte reads
   * correspond to a single sampling instant by checking the Data Ready interrupt.
   *
   * Each 16-bit accelerometer measurement has a full scale defined in ACCEL_FS
   * (Register 28). For each full scale setting, the accelerometers' sensitivity
   * per LSB in ACCEL_xOUT is shown in the table below:
   *
   * <pre>
   * AFS_SEL | Full Scale Range | LSB Sensitivity
   * --------+------------------+----------------
   * 0       | +/- 2g           | 8192 LSB/mg
   * 1       | +/- 4g           | 4096 LSB/mg
   * 2       | +/- 8g           | 2048 LSB/mg
   * 3       | +/- 16g          | 1024 LSB/mg
   * </pre>
   *
   * @param x 16-bit signed integer container for X-axis acceleration
   * @param y 16-bit signed integer container for Y-axis acceleration
   * @param z 16-bit signed integer container for Z-axis acceleration
   * @see MPU9250_RA_GYRO_XOUT_H
   */
pub fn get_acceleration(&mut self) -> Result<AccelerometerData> {
  let mut buffer = [0; 6];
  i2c::read_bytes(self.dev_address, RA_ACCEL_XOUT_H, 6, &mut buffer)?;
  Ok(AccelerometerData {
      x : ((buffer[0] as i16) << 8) | buffer[1] as i16,
      y : ((buffer[2] as i16) << 8) | buffer[3] as i16,
      z : ((buffer[4] as i16) << 8) | buffer[5] as i16,
  })
}

  /** Get X-axis accelerometer reading.
   * @return X-axis acceleration measurement in 16-bit 2's complement format
   * @see getMotion6()
   * @see MPU9250_RA_ACCEL_XOUT_H
   */
  pub fn get_acceleration_x(&mut self) -> Result<i16> {
      let mut buffer = [0; 2];
      i2c::read_bytes(self.dev_address, RA_ACCEL_XOUT_H, 2, &mut buffer)?;
      return Ok(((buffer[0] as i16) << 8) | buffer[1] as i16);
  }

  /** Get Y-axis accelerometer reading.
   * @return Y-axis acceleration measurement in 16-bit 2's complement format
   * @see getMotion6()
   * @see MPU9250_RA_ACCEL_YOUT_H
   */
  pub fn get_acceleration_y(&mut self) -> Result<i16> {
      let mut buffer = [0; 2];
      i2c::read_bytes(self.dev_address, RA_ACCEL_YOUT_H, 2, &mut buffer)?;
      return Ok(((buffer[0] as i16) << 8) | buffer[1] as i16);
  }

  /** Get Z-axis accelerometer reading.
   * @return Z-axis acceleration measurement in 16-bit 2's complement format
   * @see getMotion6()
   * @see MPU9250_RA_ACCEL_ZOUT_H
   */
  pub fn get_acceleration_z(&mut self) -> Result<i16> {
      let mut buffer = [0; 2];
      i2c::read_bytes(self.dev_address, RA_ACCEL_ZOUT_H, 2, &mut buffer)?;
      return Ok(((buffer[0] as i16) << 8) | buffer[1] as i16);
  }

  // TEMP_OUT_* registers
  
  /** Get current internal temperature.
   * @return Temperature reading in 16-bit 2's complement format
   * @see MPU9250_RA_TEMP_OUT_H
   */
  pub fn get_temperature(&mut self) -> Result<i16> {
      let mut buffer = [0; 2];
      i2c::read_bytes(self.dev_address, RA_TEMP_OUT_H, 2, &mut buffer)?;
      return Ok(((buffer[0] as i16) << 8) | buffer[1] as i16);
  }


  // GYRO_*OUT_* registers
  
  /** Get 3-axis gyroscope readings.
   * These gyroscope measurement registers, along with the accelerometer
   * measurement registers, temperature measurement registers, and external sensor
   * data registers, are composed of two sets of registers: an internal register
   * set and a user-facing read register set.
   * The data within the gyroscope sensors' internal register set is always
   * updated at the Sample Rate. Meanwhile, the user-facing read register set
   * duplicates the internal register set's data values whenever the serial
   * interface is idle. This guarantees that a burst read of sensor registers will
   * read measurements from the same sampling instant. Note that if burst reads
   * are not used, the user is responsible for ensuring a set of single byte reads
   * correspond to a single sampling instant by checking the Data Ready interrupt.
   *
   * Each 16-bit gyroscope measurement has a full scale defined in FS_SEL
   * (Register 27). For each full scale setting, the gyroscopes' sensitivity per
   * LSB in GYRO_xOUT is shown in the table below:
   *
   * <pre>
   * FS_SEL | Full Scale Range   | LSB Sensitivity
   * -------+--------------------+----------------
   * 0      | +/- 250 degrees/s  | 131 LSB/deg/s
   * 1      | +/- 500 degrees/s  | 65.5 LSB/deg/s
   * 2      | +/- 1000 degrees/s | 32.8 LSB/deg/s
   * 3      | +/- 2000 degrees/s | 16.4 LSB/deg/s
   * </pre>
   *
   * @see getMotion6()
   * @see MPU9250_RA_GYRO_XOUT_H
   */
pub fn get_rotation(&mut self) -> Result<GyroscopeData> {
  let mut buffer = [0; 6];
  i2c::read_bytes(self.dev_address, RA_GYRO_XOUT_H, 6, &mut buffer)?;
  Ok(GyroscopeData {
      x : ((buffer[0] as i16) << 8) | buffer[1] as i16,
      y : ((buffer[2] as i16) << 8) | buffer[3] as i16,
      z : ((buffer[4] as i16) << 8) | buffer[5] as i16,
  })
}
  /** Get X-axis gyroscope reading.
   * @return X-axis rotation measurement in 16-bit 2's complement format
   * @see getMotion6()
   * @see MPU9250_RA_GYRO_XOUT_H
   */
  pub fn get_rotation_x(&mut self) -> Result<i16> {
      let mut buffer = [0; 2];
      i2c::read_bytes(self.dev_address, RA_GYRO_XOUT_H, 2, &mut buffer)?;
      return Ok(((buffer[0] as i16) << 8) | buffer[1] as i16);
  }
  /** Get Y-axis gyroscope reading.
   * @return Y-axis rotation measurement in 16-bit 2's complement format
   * @see getMotion6()
   * @see MPU9250_RA_GYRO_YOUT_H
   */
  pub fn get_rotation_y(&mut self) -> Result<i16> {
      let mut buffer = [0; 2];
      i2c::read_bytes(self.dev_address, RA_GYRO_YOUT_H, 2, &mut buffer)?;
      return Ok(((buffer[0] as i16) << 8) | buffer[1] as i16);
  }
  /** Get Z-axis gyroscope reading.
   * @return Z-axis rotation measurement in 16-bit 2's complement format
   * @see getMotion6()
   * @see MPU9250_RA_GYRO_ZOUT_H
   */
  pub fn get_rotation_z(&mut self) -> Result<i16> {
      let mut buffer = [0; 2];
      i2c::read_bytes(self.dev_address, RA_GYRO_ZOUT_H, 2, &mut buffer)?;
      return Ok(((buffer[0] as i16) << 8) | buffer[1] as i16);
  }



  // EXT_SENS_DATA_* registers
  
  /** Read single byte from external sensor data register.
   * These registers store data read from external sensors by the Slave 0, 1, 2,
   * and 3 on the auxiliary I2C interface. Data read by Slave 4 is stored in
   * I2C_SLV4_DI (Register 53).
   *
   * External sensor data is written to these registers at the Sample Rate as
   * defined in Register 25. This access rate can be reduced by using the Slave
   * Delay Enable registers (Register 103).
   *
   * External sensor data registers, along with the gyroscope measurement
   * registers, accelerometer measurement registers, and temperature measurement
   * registers, are composed of two sets of registers: an internal register set
   * and a user-facing read register set.
   *
   * The data within the external sensors' internal register set is always updated
   * at the Sample Rate (or the reduced access rate) whenever the serial interface
   * is idle. This guarantees that a burst read of sensor registers will read
   * measurements from the same sampling instant. Note that if burst reads are not
   * used, the user is responsible for ensuring a set of single byte reads
   * correspond to a single sampling instant by checking the Data Ready interrupt.
   *
   * Data is placed in these external sensor data registers according to
   * I2C_SLV0_CTRL, I2C_SLV1_CTRL, I2C_SLV2_CTRL, and I2C_SLV3_CTRL (Registers 39,
   * 42, 45, and 48). When more than zero bytes are read (I2C_SLVx_LEN > 0) from
   * an enabled slave (I2C_SLVx_EN = 1), the slave is read at the Sample Rate (as
   * defined in Register 25) or delayed rate (if specified in Register 52 and
   * 103). During each Sample cycle, slave reads are performed in order of Slave
   * number. If all slaves are enabled with more than zero bytes to be read, the
   * order will be Slave 0, followed by Slave 1, Slave 2, and Slave 3.
   *
   * Each enabled slave will have EXT_SENS_DATA registers associated with it by
   * number of bytes read (I2C_SLVx_LEN) in order of slave number, starting from
   * EXT_SENS_DATA_00. Note that this means enabling or disabling a slave may
   * change the higher numbered slaves' associated registers. Furthermore, if
   * fewer total bytes are being read from the external sensors as a result of
   * such a change, then the data remaining in the registers which no longer have
   * an associated slave device (i.e. high numbered registers) will remain in
   * these previously allocated registers unless reset.
   *
   * If the sum of the read lengths of all SLVx transactions exceed the number of
   * available EXT_SENS_DATA registers, the excess bytes will be dropped. There
   * are 24 EXT_SENS_DATA registers and hence the total read lengths between all
   * the slaves cannot be greater than 24 or some bytes will be lost.
   *
   * Note: Slave 4's behavior is distinct from that of Slaves 0-3. For further
   * information regarding the characteristics of Slave 4, please refer to
   * Registers 49 to 53.
   *
   * EXAMPLE:
   * Suppose that Slave 0 is enabled with 4 bytes to be read (I2C_SLV0_EN = 1 and
   * I2C_SLV0_LEN = 4) while Slave 1 is enabled with 2 bytes to be read so that
   * I2C_SLV1_EN = 1 and I2C_SLV1_LEN = 2. In such a situation, EXT_SENS_DATA _00
   * through _03 will be associated with Slave 0, while EXT_SENS_DATA _04 and 05
   * will be associated with Slave 1. If Slave 2 is enabled as well, registers
   * starting from EXT_SENS_DATA_06 will be allocated to Slave 2.
   *
   * If Slave 2 is disabled while Slave 3 is enabled in this same situation, then
   * registers starting from EXT_SENS_DATA_06 will be allocated to Slave 3
   * instead.
   *
   * REGISTER ALLOCATION FOR DYNAMIC DISABLE VS. NORMAL DISABLE:
   * If a slave is disabled at any time, the space initially allocated to the
   * slave in the EXT_SENS_DATA register, will remain associated with that slave.
   * This is to avoid dynamic adjustment of the register allocation.
   *
   * The allocation of the EXT_SENS_DATA registers is recomputed only when (1) all
   * slaves are disabled, or (2) the I2C_MST_RST bit is set (Register 106).
   *
   * This above is also true if one of the slaves gets NACKed and stops
   * functioning.
   *
   * @param position Starting position (0-23)
   * @return Byte read from register
   */
pub fn get_external_sensor_byte(&mut self, position: u8) -> Result<u8> {
    if position > 23 {
        return Err(anyhow!("Invalid Parameter"));
    }
    i2c::read_byte(self.dev_address, RA_EXT_SENS_DATA_00 + position)
}

/// Read word (2 bytes) from external sensor data registers.
///
/// # Arguments
/// * `position` - Starting position (0-21)
///
/// # Returns
/// A `Result<u16>` representing the word read from the register.
pub fn get_external_sensor_word(&mut self, position: u8) -> Result<u16> {
    if position > 21 {
        return Err(anyhow!("Invalid Parameter"));
    }
    let mut buffer = [0u8; 2];
    i2c::read_bytes(self.dev_address, RA_EXT_SENS_DATA_00 + position, 2, &mut buffer)?;
    Ok(u16::from_be_bytes(buffer))
}

/// Read double word (4 bytes) from external sensor data registers.
///
/// # Arguments
/// * `position` - Starting position (0-20)
///
/// # Returns
/// A `Result<u32>` representing the double word read from the registers.
pub fn get_external_sensor_dword(&mut self, position: u8) -> Result<u32> {
    if position > 20 {
        return Err(anyhow!("Invalid Parameter"));
    }
    let mut buffer = [0u8; 4];
    i2c::read_bytes(self.dev_address, RA_EXT_SENS_DATA_00 + position, 4, &mut buffer)?;
    Ok(u32::from_be_bytes(buffer))
}

  // MOT_DETECT_STATUS register
  
  /** Get X-axis negative motion detection interrupt status.
   * @return Motion detection status
   * @see MPU9250_RA_MOT_DETECT_STATUS
   * @see MPU9250_MOTION_MOT_XNEG_BIT
   */
pub fn get_x_neg_motion_detected(&mut self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_MOT_DETECT_STATUS, MOTION_MOT_XNEG_BIT)
}

  /** Get X-axis positive motion detection interrupt status.
   * @return Motion detection status
   * @see MPU9250_RA_MOT_DETECT_STATUS
   * @see MPU9250_MOTION_MOT_XPOS_BIT
   */
pub fn get_x_pos_motion_detected(&mut self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_MOT_DETECT_STATUS, MOTION_MOT_XPOS_BIT)
}

  /** Get Y-axis negative motion detection interrupt status.
   * @return Motion detection status
   * @see MPU9250_RA_MOT_DETECT_STATUS
   * @see MPU9250_MOTION_MOT_YNEG_BIT
   */
pub fn get_y_neg_motion_detected(&mut self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_MOT_DETECT_STATUS, MOTION_MOT_YNEG_BIT)
}

  /** Get Y-axis positive motion detection interrupt status.
   * @return Motion detection status
   * @see MPU9250_RA_MOT_DETECT_STATUS
   * @see MPU9250_MOTION_MOT_YPOS_BIT
   */
pub fn get_y_pos_motion_detected(&mut self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_MOT_DETECT_STATUS, MOTION_MOT_YPOS_BIT)
}

  /** Get Z-axis negative motion detection interrupt status.
   * @return Motion detection status
   * @see MPU9250_RA_MOT_DETECT_STATUS
   * @see MPU9250_MOTION_MOT_ZNEG_BIT
   */
pub fn get_z_neg_motion_detected(&mut self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_MOT_DETECT_STATUS, MOTION_MOT_ZNEG_BIT)
}

  /** Get Z-axis positive motion detection interrupt status.
   * @return Motion detection status
   * @see MPU9250_RA_MOT_DETECT_STATUS
   * @see MPU9250_MOTION_MOT_ZPOS_BIT
   */
pub fn get_z_pos_motion_detected(&mut self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_MOT_DETECT_STATUS, MOTION_MOT_ZPOS_BIT)
}

  /** Get zero motion detection interrupt status.
   * @return Motion detection status
   * @see MPU9250_RA_MOT_DETECT_STATUS
   * @see MPU9250_MOTION_MOT_ZRMOT_BIT
   */
pub fn get_zero_motion_detected(&mut self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_MOT_DETECT_STATUS, MOTION_MOT_ZRMOT_BIT)
}

  // I2C_SLV*_DO register
  
  /** Write byte to Data Output container for specified slave.
   * This register holds the output data written into Slave when Slave is set to
   * write mode. For further information regarding Slave control, please
   * refer to Registers 37 to 39 and immediately following.
   * @param num Slave number (0-3)
   * @param data Byte to write
   * @see MPU9250_RA_I2C_SLV0_DO
   */
pub fn set_slave_output_byte(&mut self, num: u8, data: u8) -> Result<()> {
    if num > 3 {
        return Err(anyhow!("Invalid Parameter"));
    }
    i2c::write_byte(self.dev_address, RA_I2C_SLV0_DO + num, data)
}

  // I2C_MST_DELAY_CTRL register

  /** Get external data shadow delay enabled status.
   * This register is used to specify the timing of external sensor data
   * shadowing. When DELAY_ES_SHADOW is set to 1, shadowing of external
   * sensor data is delayed until all data has been received.
   * @return Current external data shadow delay enabled status.
   * @see MPU9250_RA_I2C_MST_DELAY_CTRL
   * @see MPU9250_DELAYCTRL_DELAY_ES_SHADOW_BIT
   */
pub fn get_external_shadow_delay_enabled(&mut self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_I2C_MST_DELAY_CTRL, DELAYCTRL_DELAY_ES_SHADOW_BIT)
}

  /** Set external data shadow delay enabled status.
   * @param enabled New external data shadow delay enabled status.
   * @see getExternalShadowDelayEnabled()
   * @see MPU9250_RA_I2C_MST_DELAY_CTRL
   * @see MPU9250_DELAYCTRL_DELAY_ES_SHADOW_BIT
   */
pub fn set_external_shadow_delay_enabled(&mut self, enabled: bool) -> Result<()> {
    i2c::write_bit(self.dev_address, RA_I2C_MST_DELAY_CTRL, DELAYCTRL_DELAY_ES_SHADOW_BIT, enabled as u8)
}

  /** Get slave delay enabled status.
   * When a particular slave delay is enabled, the rate of access for the that
   * slave device is reduced. When a slave's access rate is decreased relative to
   * the Sample Rate, the slave is accessed every:
   *
   *    //```no_run
   *     1 / (1 + RA_I2C_MST_DELAY_CTRL); // Samples
   *    //```
   *
   * This base Sample Rate in turn is determined by SMPLRT_DIV (register  * 25)
   * and DLPF_CFG (register 26).
   *
   * For further information regarding I2C_MST_DLY, please refer to register 52.
   * For further information regarding the Sample Rate, please refer to register 25.
   *
   * @param num Slave number (0-4)
   * @return Current slave delay enabled status.
   * @see MPU9250_RA_I2C_MST_DELAY_CTRL
   * @see MPU9250_DELAYCTRL_I2C_SLV0_DLY_EN_BIT
   */
pub fn get_slave_delay_enabled(&mut self, num: u8) -> Result<u8> {
    if num > 4 {
        return Err(anyhow!("Invalid Parameter"));
    }
    i2c::read_bit(self.dev_address, RA_I2C_MST_DELAY_CTRL, num)
}

  /** Set slave delay enabled status.
   * @param num Slave number (0-4)
   * @param enabled New slave delay enabled status.
   * @see MPU9250_RA_I2C_MST_DELAY_CTRL
   * @see MPU9250_DELAYCTRL_I2C_SLV0_DLY_EN_BIT
   */
pub fn set_slave_delay_enabled(&mut self, num: u8, enabled: bool) -> Result<()> {
    if num > 4 {
        return Err(anyhow!("Invalid Parameter"));
    }
    i2c::write_bit(self.dev_address, RA_I2C_MST_DELAY_CTRL, num, enabled as u8)
}

  /** Reset gyroscope signal path.
   * The reset will revert the signal path analog to digital converters and
   * filters to their power up configurations.
   * @see MPU9250_RA_SIGNAL_PATH_RESET
   * @see MPU9250_PATHRESET_GYRO_RESET_BIT
   */
pub fn reset_gyroscope_path(&mut self) -> Result<()> {
    i2c::write_bit(self.dev_address, RA_SIGNAL_PATH_RESET, PATHRESET_GYRO_RESET_BIT, true as u8)
}

  /** Reset accelerometer signal path.
   * The reset will revert the signal path analog to digital converters and
   * filters to their power up configurations.
   * @see MPU9250_RA_SIGNAL_PATH_RESET
   * @see MPU9250_PATHRESET_ACCEL_RESET_BIT
   */
pub fn reset_accelerometer_path(&mut self) -> Result<()> {
    i2c::write_bit(self.dev_address, RA_SIGNAL_PATH_RESET, PATHRESET_ACCEL_RESET_BIT, true as u8)
}

  /** Reset temperature sensor signal path.
   * The reset will revert the signal path analog to digital converters and
   * filters to their power up configurations.
   * @see MPU9250_RA_SIGNAL_PATH_RESET
   * @see MPU9250_PATHRESET_TEMP_RESET_BIT
   */
pub fn reset_temperature_path(&mut self) -> Result<()> {
    i2c::write_bit(self.dev_address, RA_SIGNAL_PATH_RESET, PATHRESET_TEMP_RESET_BIT, true as u8)
}

  /** Get accelerometer power-on delay.
   * The accelerometer data path provides samples to the sensor registers, Motion
   * detection, Zero Motion detection, and Free Fall detection modules. The
   * signal path contains filters which must be flushed on wake-up with new
   * samples before the detection modules begin operations. The default wake-up
   * delay, of 4ms can be lengthened by up to 3ms. This additional delay is
   * specified in ACCEL_ON_DELAY in units of 1 LSB = 1 ms. The user may select
   * any value above zero unless instructed otherwise by InvenSense. Please refer
   * to Section 8 of the MPU-6000/MPU-9250 Product Specification document for
   * further information regarding the detection modules.
   * @return Current accelerometer power-on delay
   * @see MPU9250_RA_MOT_DETECT_CTRL
   * @see MPU9250_DETECT_ACCEL_ON_DELAY_BIT
   */
pub fn get_accelerometer_power_on_delay(&mut self) -> Result<u8> {
    i2c::read_bits(
        self.dev_address,
        RA_MOT_DETECT_CTRL,
        DETECT_ACCEL_ON_DELAY_BIT,
        DETECT_ACCEL_ON_DELAY_LENGTH,
    )
}

  /** Set accelerometer power-on delay.
   * @param delay New accelerometer power-on delay (0-3)
   * @see getAccelerometerPowerOnDelay()
   * @see MPU9250_RA_MOT_DETECT_CTRL
   * @see MPU9250_DETECT_ACCEL_ON_DELAY_BIT
   */
pub fn set_accelerometer_power_on_delay(&mut self, delay: u8) -> Result<()> {
    if delay > 3 {
        return Err(anyhow!("Invalid Parameter"));
    }
    i2c::write_bits(
        self.dev_address,
        RA_MOT_DETECT_CTRL,
        DETECT_ACCEL_ON_DELAY_BIT,
        DETECT_ACCEL_ON_DELAY_LENGTH,
        delay,
    )
}

  /** Get Free Fall detection counter decrement configuration.
   * Detection is registered by the Free Fall detection module after accelerometer
   * measurements meet their respective threshold conditions over a specified
   * number of samples. When the threshold conditions are met, the corresponding
   * detection counter increments by 1. The user may control the rate at which the
   * detection counter decrements when the threshold condition is not met by
   * configuring FF_COUNT. The decrement rate can be set according to the
   * following table:
   *
   * <pre>
   * FF_COUNT | Counter Decrement
   * ---------+------------------
   * 0        | Reset
   * 1        | 1
   * 2        | 2
   * 3        | 4
   * </pre>
   *
   * When FF_COUNT is configured to 0 (reset), any non-qualifying sample will
   * reset the counter to 0. For further information on Free Fall detection,
   * please refer to Registers 29 to 32.
   *
   * @return Current decrement configuration
   * @see MPU9250_RA_MOT_DETECT_CTRL
   * @see MPU9250_DETECT_FF_COUNT_BIT
   */
pub fn get_freefall_detection_counter_decrement(&mut self) -> Result<u8> {
    i2c::read_bits(
        self.dev_address,
        RA_MOT_DETECT_CTRL,
        DETECT_FF_COUNT_BIT,
        DETECT_FF_COUNT_LENGTH,
    )
}

  /** Set Free Fall detection counter decrement configuration.
   * @param decrement New decrement configuration value
   * @see getFreefallDetectionCounterDecrement()
   * @see MPU9250_RA_MOT_DETECT_CTRL
   * @see MPU9250_DETECT_FF_COUNT_BIT
   */
pub fn set_freefall_detection_counter_decrement(&mut self, decrement: u8) -> Result<()> {
    if decrement > 3 {
        return Err(anyhow!("Invalid Parameter"));
    }
    i2c::write_bits(
        self.dev_address,
        RA_MOT_DETECT_CTRL,
        DETECT_FF_COUNT_BIT,
        DETECT_FF_COUNT_LENGTH,
        decrement,
    )
}

  /** Get Motion detection counter decrement configuration.
   * Detection is registered by the Motion detection module after accelerometer
   * measurements meet their respective threshold conditions over a specified
   * number of samples. When the threshold conditions are met, the corresponding
   * detection counter increments by 1. The user may control the rate at which the
   * detection counter decrements when the threshold condition is not met by
   * configuring MOT_COUNT. The decrement rate can be set according to the
   * following table:
   *
   * <pre>
   * MOT_COUNT | Counter Decrement
   * ----------+------------------
   * 0         | Reset
   * 1         | 1
   * 2         | 2
   * 3         | 4
   * </pre>
   *
   * When MOT_COUNT is configured to 0 (reset), any non-qualifying sample will
   * reset the counter to 0. For further information on Motion detection,
   * please refer to Registers 29 to 32.
   *
   */
pub fn get_motion_detection_counter_decrement(&mut self) -> Result<u8> {
    i2c::read_bits(
        self.dev_address,
        RA_MOT_DETECT_CTRL,
        DETECT_MOT_COUNT_BIT,
        DETECT_MOT_COUNT_LENGTH,
    )
}

  /** Set Motion detection counter decrement configuration.
   * @param decrement New decrement configuration value
   * @see getMotionDetectionCounterDecrement()
   * @see MPU9250_RA_MOT_DETECT_CTRL
   * @see MPU9250_DETECT_MOT_COUNT_BIT
   */
pub fn set_motion_detection_counter_decrement(&mut self, decrement: u8) -> Result<()> {
    if decrement > 3 {
        return Err(anyhow!("Invalid Parameter"));
    }
    i2c::write_bits(
        self.dev_address,
        RA_MOT_DETECT_CTRL,
        DETECT_MOT_COUNT_BIT,
        DETECT_MOT_COUNT_LENGTH,
        decrement,
    )
}

  /** Get FIFO enabled status.
   * When this bit is set to 0, the FIFO buffer is disabled. The FIFO buffer
   * cannot be written to or read from while disabled. The FIFO buffer's state
   * does not change unless the MPU-60X0 is power cycled.
   * @return Current FIFO enabled status
   * @see MPU9250_RA_USER_CTRL
   * @see MPU9250_USERCTRL_FIFO_EN_BIT
   */
pub fn get_fifo_enabled(&mut self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_USER_CTRL, USERCTRL_FIFO_EN_BIT)
}

  /** Set FIFO enabled status.
   * @param enabled New FIFO enabled status
   * @see getFIFOEnabled()
   * @see MPU9250_RA_USER_CTRL
   * @see MPU9250_USERCTRL_FIFO_EN_BIT
   */
pub fn set_fifo_enabled(&mut self, enabled: bool) -> Result<()> {
    i2c::write_bit(self.dev_address, RA_USER_CTRL, USERCTRL_FIFO_EN_BIT, enabled as u8)
}

  /** Get I2C Master Mode enabled status.
   * When this mode is enabled, the MPU-60X0 acts as the I2C Master to the
   * external sensor slave devices on the auxiliary I2C bus. When this bit is
   * cleared to 0, the auxiliary I2C bus lines (AUX_DA and AUX_CL) are logically
   * driven by the primary I2C bus (SDA and SCL). This is a precondition to
   * enabling Bypass Mode. For further information regarding Bypass Mode, please
   * refer to Register 55.
   * @return Current I2C Master Mode enabled status
   * @see MPU9250_RA_USER_CTRL
   * @see MPU9250_USERCTRL_I2C_MST_EN_BIT
   */
pub fn get_i2c_master_mode_enabled(&mut self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_USER_CTRL, USERCTRL_I2C_MST_EN_BIT)
}

  /** Set I2C Master Mode enabled status.
   * @param enabled New I2C Master Mode enabled status
   * @see getI2CMasterModeEnabled()
   * @see MPU9250_RA_USER_CTRL
   * @see MPU9250_USERCTRL_I2C_MST_EN_BIT
   */
pub fn set_i2c_master_mode_enabled(&mut self, enabled: bool) -> Result<()> {
    i2c::write_bit(self.dev_address, RA_USER_CTRL, USERCTRL_I2C_MST_EN_BIT, enabled as u8)
}

  /** Switch from I2C to SPI mode (MPU-6000 only)
   * If this is set, the primary SPI interface will be enabled in place of the
   * disabled primary I2C interface.
   */
pub fn switch_spi_enabled(&mut self, enabled: bool) -> Result<()> {
    i2c::write_bit(self.dev_address, RA_USER_CTRL, USERCTRL_I2C_IF_DIS_BIT, enabled as u8)
}

  /** Reset the FIFO.
   * This bit resets the FIFO buffer when set to 1 while FIFO_EN equals 0. This
   * bit automatically clears to 0 after the reset has been triggered.
   * @see MPU9250_RA_USER_CTRL
   * @see MPU9250_USERCTRL_FIFO_RESET_BIT
   */
pub fn reset_fifo(&mut self) -> Result<()> {
    i2c::write_bit(self.dev_address, RA_USER_CTRL, USERCTRL_FIFO_RESET_BIT, 1)
}

  /** Reset the I2C Master.
   * This bit resets the I2C Master when set to 1 while I2C_MST_EN equals 0.
   * This bit automatically clears to 0 after the reset has been triggered.
   * @see MPU9250_RA_USER_CTRL
   * @see MPU9250_USERCTRL_I2C_MST_RESET_BIT
   */
pub fn reset_i2c_master(&mut self) -> Result<()> {
    i2c::write_bit(self.dev_address, RA_USER_CTRL, USERCTRL_I2C_MST_RESET_BIT, 1)
}

  /** Reset all sensor registers and signal paths.
   * When set to 1, this bit resets the signal paths for all sensors (gyroscopes,
   * accelerometers, and temperature sensor). This operation will also clear the
   * sensor registers. This bit automatically clears to 0 after the reset has been
   * triggered.
   *
   * When resetting only the signal path (and not the sensor registers), please
   * use Register 104, SIGNAL_PATH_RESET.
   *
   * @see MPU9250_RA_USER_CTRL
   * @see MPU9250_USERCTRL_SIG_COND_RESET_BIT
   */
pub fn reset_sensors(&mut self) -> Result<()> {
    i2c::write_bit(self.dev_address, RA_USER_CTRL, USERCTRL_SIG_COND_RESET_BIT, 1)
}

  /** Trigger a full device reset.
   * A small delay of ~50ms may be desirable after triggering a reset.
   * @see MPU9250_RA_PWR_MGMT_1
   * @see MPU9250_PWR1_DEVICE_RESET_BIT
   */
pub fn reset(&mut self) -> Result<()> {
    i2c::write_bit(self.dev_address, RA_PWR_MGMT_1, PWR1_DEVICE_RESET_BIT, 1)
}

  /** Get sleep mode status.
   * Setting the SLEEP bit in the register puts the device into very low power
   * sleep mode. In this mode, only the serial interface and internal registers
   * remain active, allowing for a very low standby current. Clearing this bit
   * puts the device back into normal mode. To save power, the individual standby
   * selections for each of the gyros should be used if any gyro axis is not used
   * by the application.
   * @return Current sleep mode enabled status
   * @see MPU9250_RA_PWR_MGMT_1
   * @see MPU9250_PWR1_SLEEP_BIT
   */
pub fn get_sleep_enabled(&mut self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_PWR_MGMT_1, PWR1_SLEEP_BIT)
}

  /** Set sleep mode status.
   * @param enabled New sleep mode enabled status
   * @see getSleepEnabled()
   * @see MPU9250_RA_PWR_MGMT_1
   * @see MPU9250_PWR1_SLEEP_BIT
   */
pub fn set_sleep_enabled(&mut self, enabled: bool) -> Result<()> {
    i2c::write_bit(self.dev_address, RA_PWR_MGMT_1, PWR1_SLEEP_BIT, enabled as u8)
}

pub fn set_magnetometer_enabled(&mut self, enabled: bool) -> Result<()> {
    i2c::write_byte(mpu9150::RA_MAG_ADDRESS, mpu9150::RA_MAG_CTRL, mpu9150::RA_MAG_CTRL_SNGL_MSR)
}

  /** Get wake cycle enabled status.
   * When this bit is set to 1 and SLEEP is disabled, the MPU-60X0 will cycle
   * between sleep mode and waking up to take a single sample of data from active
   * sensors at a rate determined by LP_WAKE_CTRL (register 108).
   * @return Current sleep mode enabled status
   * @see MPU9250_RA_PWR_MGMT_1
   * @see MPU9250_PWR1_CYCLE_BIT
   */
pub fn get_wake_cycle_enabled(&mut self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_PWR_MGMT_1, PWR1_CYCLE_BIT)
}

  /** Set wake cycle enabled status.
   * @param enabled New sleep mode enabled status
   * @see getWakeCycleEnabled()
   * @see MPU9250_RA_PWR_MGMT_1
   * @see MPU9250_PWR1_CYCLE_BIT
   */
pub fn set_wake_cycle_enabled(&mut self, enabled: bool) -> Result<()> {
    i2c::write_bit(self.dev_address, RA_PWR_MGMT_1, PWR1_CYCLE_BIT, enabled as u8)
}

  /** Get temperature sensor enabled status.
   * Control the usage of the internal temperature sensor.
   *
   * Note: this register stores the *disabled* value, but for consistency with the
   * rest of the code, the function is named and used with standard true/false
   * values to indicate whether the sensor is enabled or disabled, respectively.
   *
   * @return Current temperature sensor enabled status
   * @see MPU9250_RA_PWR_MGMT_1
   * @see MPU9250_PWR1_TEMP_DIS_BIT
   */
pub fn get_temp_sensor_enabled(&mut self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_PWR_MGMT_1, PWR1_TEMP_DIS_BIT)
}

  /** Set temperature sensor enabled status.
   * Note: this register stores the *disabled* value, but for consistency with the
   * rest of the code, the function is named and used with standard true/false
   * values to indicate whether the sensor is enabled or disabled, respectively.
   *
   * @param enabled New temperature sensor enabled status
   * @see getTempSensorEnabled()
   * @see MPU9250_RA_PWR_MGMT_1
   * @see MPU9250_PWR1_TEMP_DIS_BIT
   */
pub fn set_temp_sensor_enabled(&mut self, enabled: bool) -> Result<()> {
    i2c::write_bit(self.dev_address, RA_PWR_MGMT_1, PWR1_TEMP_DIS_BIT, (!enabled) as u8)
}

  /** Get clock source setting.
   * @return Current clock source setting
   * @see MPU9250_RA_PWR_MGMT_1
   * @see MPU9250_PWR1_CLKSEL_BIT
   * @see MPU9250_PWR1_CLKSEL_LENGTH
   */
pub fn get_clock_source(&mut self) -> Result<u8> {
    i2c::read_bits(self.dev_address, RA_PWR_MGMT_1, PWR1_CLKSEL_BIT, PWR1_CLKSEL_LENGTH)
}

/** Set clock source setting.
* An internal 8MHz oscillator, gyroscope based clock, or external sources can
* be selected as the MPU-60X0 clock source. When the internal 8 MHz oscillator
* or an external source is chosen as the clock source, the MPU-60X0 can operate
* in low power modes with the gyroscopes disabled.
*
* Upon power up, the MPU-60X0 clock source defaults to the internal oscillator.
* However, it is highly recommended that the device be configured to use one of
* the gyroscopes (or an external clock source) as the clock reference for
* improved stability. The clock source can be selected according to the following table:
*
* <pre>
* CLK_SEL | Clock Source
* --------+--------------------------------------
* 0       | Internal oscillator
* 1       | PLL with X Gyro reference
* 2       | PLL with Y Gyro reference
* 3       | PLL with Z Gyro reference
* 4       | PLL with external 32.768kHz reference
* 5       | PLL with external 19.2MHz reference
* 6       | Reserved
* 7       | Stops the clock and keeps the timing generator in reset
* </pre>
*
* @param source New clock source setting
* @see getClockSource()
* @see MPU9250_RA_PWR_MGMT_1
* @see MPU9250_PWR1_CLKSEL_BIT
* @see MPU9250_PWR1_CLKSEL_LENGTH
*/
pub fn set_clock_source(&mut self, source: u8) -> Result<()> {
    i2c::write_bits(self.dev_address, RA_PWR_MGMT_1, PWR1_CLKSEL_BIT, PWR1_CLKSEL_LENGTH, source)
}

// PWR_MGMT_2 register

/** Get wake frequency in Accel-Only Low Power Mode.
 * The MPU-60X0 can be put into Accerlerometer Only Low Power Mode by setting
 * PWRSEL to 1 in the Power Management 1 register (Register 107). In this mode,
 * the device will power off all devices except for the primary I2C interface,
 * waking only the accelerometer at fixed intervals to take a single
 * measurement. The frequency of wake-ups can be configured with LP_WAKE_CTRL
 * as shown below:
 *
 * <pre>
 * LP_WAKE_CTRL | Wake-up Frequency
 * -------------+------------------
 * 0            | 1.25 Hz
 * 1            | 2.5 Hz
 * 2            | 5 Hz
 * 3            | 10 Hz
 * <pre>
 *
 * For further information regarding the MPU-60X0's power modes, please refer to
 * Register 107.
 *
 * @return Current wake frequency
 * @see MPU9250_RA_PWR_MGMT_2
 */
pub fn get_wake_frequency(&mut self) -> Result<u8> {
    i2c::read_bits(self.dev_address, RA_PWR_MGMT_2, PWR2_LP_WAKE_CTRL_BIT, PWR2_LP_WAKE_CTRL_LENGTH)
        .map(|val| val as u8)
}

/** Set wake frequency in Accel-Only Low Power Mode.
 * @param frequency New wake frequency
 * @see MPU9250_RA_PWR_MGMT_2
 */
pub fn set_wake_frequency(&mut self, frequency: u8) -> Result<()> {
    i2c::write_bits(self.dev_address, RA_PWR_MGMT_2, PWR2_LP_WAKE_CTRL_BIT, PWR2_LP_WAKE_CTRL_LENGTH, frequency)
}

/** Get X-axis accelerometer standby enabled status.
 * If enabled, the X-axis will not gather or report data (or use power).
 * @return Current X-axis standby enabled status
 * @see MPU9250_RA_PWR_MGMT_2
 * @see MPU9250_PWR2_STBY_XA_BIT
 */
pub fn get_standby_x_accel_enabled(&self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_PWR_MGMT_2, PWR2_STBY_XA_BIT)
}

/** Set X-axis accelerometer standby enabled status.
 * @param enabled New X-axis standby enabled status
 * @see get_standby_x_accel_enabled()
 * @see MPU9250_RA_PWR_MGMT_2
 * @see MPU9250_PWR2_STBY_XA_BIT
 */
pub fn set_standby_x_accel_enabled(&self, enabled: u8) -> Result<()> {
    i2c::write_bit(self.dev_address, RA_PWR_MGMT_2, PWR2_STBY_XA_BIT, enabled)
}

/** Get Y-axis accelerometer standby enabled status.
 * If enabled, the Y-axis will not gather or report data (or use power).
 * @return Current Y-axis standby enabled status
 * @see MPU9250_RA_PWR_MGMT_2
 * @see MPU9250_PWR2_STBY_YA_BIT
 */
pub fn get_standby_y_accel_enabled(&self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_PWR_MGMT_2, PWR2_STBY_YA_BIT)
}

/** Set Y-axis accelerometer standby enabled status.
 * @param enabled New Y-axis standby enabled status
 * @see get_standby_y_accel_enabled()
 * @see MPU9250_RA_PWR_MGMT_2
 * @see MPU9250_PWR2_STBY_YA_BIT
 */
pub fn set_standby_y_accel_enabled(&self, enabled: u8) -> Result<()> {
    i2c::write_bit(self.dev_address, RA_PWR_MGMT_2, PWR2_STBY_YA_BIT, enabled)
}

/** Get Z-axis accelerometer standby enabled status.
 * If enabled, the Z-axis will not gather or report data (or use power).
 * @return Current Z-axis standby enabled status
 * @see MPU9250_RA_PWR_MGMT_2
 * @see MPU9250_PWR2_STBY_ZA_BIT
 */
pub fn get_standby_z_accel_enabled(&self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_PWR_MGMT_2, PWR2_STBY_ZA_BIT)
}

/** Set Z-axis accelerometer standby enabled status.
 * @param enabled New Z-axis standby enabled status
 * @see get_standby_z_accel_enabled()
 * @see MPU9250_RA_PWR_MGMT_2
 * @see MPU9250_PWR2_STBY_ZA_BIT
 */
pub fn set_standby_z_accel_enabled(&self, enabled: u8) -> Result<()> {
    i2c::write_bit(self.dev_address, RA_PWR_MGMT_2, PWR2_STBY_ZA_BIT, enabled)
}

/** Get X-axis gyroscope standby enabled status.
 * If enabled, the X-axis will not gather or report data (or use power).
 * @return Current X-axis standby enabled status
 * @see MPU9250_RA_PWR_MGMT_2
 * @see MPU9250_PWR2_STBY_XG_BIT
 */
pub fn get_standby_x_gyro_enabled(&self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_PWR_MGMT_2, PWR2_STBY_XG_BIT)
}

/** Set X-axis gyroscope standby enabled status.
 * @param enabled New X-axis standby enabled status
 * @see get_standby_x_gyro_enabled()
 * @see MPU9250_RA_PWR_MGMT_2
 * @see MPU9250_PWR2_STBY_XG_BIT
 */
pub fn set_standby_x_gyro_enabled(&self, enabled: u8) -> Result<()> {
    i2c::write_bit(self.dev_address, RA_PWR_MGMT_2, PWR2_STBY_XG_BIT, enabled)
}

/** Get Y-axis gyroscope standby enabled status.
 * If enabled, the Y-axis will not gather or report data (or use power).
 * @return Current Y-axis standby enabled status
 * @see MPU9250_RA_PWR_MGMT_2
 * @see MPU9250_PWR2_STBY_YG_BIT
 */
pub fn get_standby_y_gyro_enabled(&self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_PWR_MGMT_2, PWR2_STBY_YG_BIT)
}

/** Set Y-axis gyroscope standby enabled status.
 * @param enabled New Y-axis standby enabled status
 * @see get_standby_y_gyro_enabled()
 * @see MPU9250_RA_PWR_MGMT_2
 * @see MPU9250_PWR2_STBY_YG_BIT
 */
pub fn set_standby_y_gyro_enabled(&self, enabled: u8) -> Result<()> {
    i2c::write_bit(self.dev_address, RA_PWR_MGMT_2, PWR2_STBY_YG_BIT, enabled)
}

/** Get Z-axis gyroscope standby enabled status.
 * If enabled, the Z-axis will not gather or report data (or use power).
 * @return Current Z-axis standby enabled status
 * @see MPU9250_RA_PWR_MGMT_2
 * @see MPU9250_PWR2_STBY_ZG_BIT
 */
pub fn get_standby_z_gyro_enabled(&self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_PWR_MGMT_2, PWR2_STBY_ZG_BIT)
}

/** Set Z-axis gyroscope standby enabled status.
 * @param enabled New Z-axis standby enabled status
 * @see get_standby_z_gyro_enabled()
 * @see MPU9250_RA_PWR_MGMT_2
 * @see MPU9250_PWR2_STBY_ZG_BIT
 */
pub fn set_standby_z_gyro_enabled(&self, enabled: u8) -> Result<()> {
    i2c::write_bit(self.dev_address, RA_PWR_MGMT_2, PWR2_STBY_ZG_BIT, enabled)
}

// FIFO_COUNT* registers

/** Get current FIFO buffer size.
 * This value indicates the number of bytes stored in the FIFO buffer. This
 * number is in turn the number of bytes that can be read from the FIFO buffer
 * and it is directly proportional to the number of samples available given the
 * set of sensor data bound to be stored in the FIFO (register 35 and 36).
 * @return Current FIFO buffer size
 */
pub fn get_fifo_count(&self) -> Result<u16> {
    let mut buffer = [0; 2];
    i2c::read_bytes(self.dev_address, RA_FIFO_COUNTH, 2, &mut buffer)?;
    Ok(((buffer[1] as u16) << 8) | buffer[0] as u16)
}

// FIFO_R_W register

/** Get byte from FIFO buffer.
 * This register is used to read and write data from the FIFO buffer. Data is
 * written to the FIFO in order of register number (from lowest to highest). If
 * all the FIFO enable flags (see below) are enabled and all External Sensor
 * Data registers (Registers 73 to 96) are associated with a Slave device, the
 * contents of registers 59 through 96 will be written in order at the Sample
 * Rate.
 *
 * The contents of the sensor data registers (Registers 59 to 96) are written
 * into the FIFO buffer when their corresponding FIFO enable flags are set to 1
 * in FIFO_EN (Register 35). An additional flag for the sensor data registers
 * associated with I2C Slave 3 can be found in I2C_MST_CTRL (Register 36).
 *
 * If the FIFO buffer has overflowed, the status bit FIFO_OFLOW_INT is
 * automatically set to 1. This bit is located in INT_STATUS (Register 58).
 * When the FIFO buffer has overflowed, the oldest data will be lost and new
 * data will be written to the FIFO.
 *
 * If the FIFO buffer is empty, reading this register will return the last byte
 * that was previously read from the FIFO until new data is available. The user
 * should check FIFO_COUNT to ensure that the FIFO buffer is not read when
 * empty.
 *
 * @return Byte from FIFO buffer
 */
pub fn get_fifo_byte(&self) -> Result<u8> {
    i2c::read_byte(self.dev_address, RA_FIFO_R_W)
}

/** Write byte to FIFO buffer.
 * @see get_fifo_byte()
 * @see MPU9250_RA_FIFO_R_W
 */
pub fn set_fifo_byte(&self, data: u8) -> Result<()> {
    i2c::write_byte(self.dev_address, RA_FIFO_R_W, data)
}

// WHO_AM_I register

/** Get Device ID.
 * This register is used to verify the identity of the device (0b110100, 0x34).
 * @return Device ID (6 bits only! should be 0x34)
 * @see MPU9250_RA_WHO_AM_I
 * @see MPU9250_WHO_AM_I_BIT
 * @see MPU9250_WHO_AM_I_LENGTH
 */
pub fn get_device_id(&self) -> Result<u8> {
    i2c::read_byte(self.dev_address, RA_WHO_AM_I)
}

/** Set Device ID.
 * Write a new ID into the WHO_AM_I register (no idea why this should ever be
 * necessary though).
 * @param id New device ID to set.
 * @see get_device_id()
 * @see MPU9250_RA_WHO_AM_I
 * @see MPU9250_WHO_AM_I_BIT
 * @see MPU9250_WHO_AM_I_LENGTH
 */
pub fn set_device_id(&self, id: u8) -> Result<()> {
    i2c::write_byte(self.dev_address, RA_WHO_AM_I, id)
}

// ======== UNDOCUMENTED/DMP REGISTERS/METHODS ========

// XG_OFFS_TC register

pub fn get_otp_bank_valid(&self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_XG_OFFS_TC, TC_OTP_BNK_VLD_BIT)
}

pub fn set_otp_bank_valid(&self, enabled: u8) -> Result<()> {
    i2c::write_bit(self.dev_address, RA_XG_OFFS_TC, TC_OTP_BNK_VLD_BIT, enabled)
}

pub fn get_x_gyro_offset(&self) -> Result<u8> {
    i2c::read_bits(self.dev_address, RA_XG_OFFS_TC, TC_OFFSET_BIT, TC_OFFSET_LENGTH)
}

pub fn set_x_gyro_offset(&self, offset: u8) -> Result<()> {
    i2c::write_bits(self.dev_address, RA_XG_OFFS_TC, TC_OFFSET_BIT, TC_OFFSET_LENGTH, offset)
}

// YG_OFFS_TC register

pub fn get_y_gyro_offset(&self) -> Result<u8> {
    i2c::read_bits(self.dev_address, RA_YG_OFFS_TC, TC_OFFSET_BIT, TC_OFFSET_LENGTH)
}

pub fn set_y_gyro_offset(&self, offset: u8) -> Result<()> {
    i2c::write_bits(self.dev_address, RA_YG_OFFS_TC, TC_OFFSET_BIT, TC_OFFSET_LENGTH, offset)
}

// ZG_OFFS_TC register

pub fn get_z_gyro_offset(&self) -> Result<u8> {
    i2c::read_bits(self.dev_address, RA_ZG_OFFS_TC, TC_OFFSET_BIT, TC_OFFSET_LENGTH)
}

pub fn set_z_gyro_offset(&self, offset: u8) -> Result<()> {
    i2c::write_bits(self.dev_address, RA_ZG_OFFS_TC, TC_OFFSET_BIT, TC_OFFSET_LENGTH, offset)
}

// X_FINE_GAIN register

pub fn get_x_fine_gain(&self) -> Result<u8> {
    i2c::read_byte(self.dev_address, RA_X_FINE_GAIN)
}

pub fn set_x_fine_gain(&self, gain: u8) -> Result<()> {
    i2c::write_byte(self.dev_address, RA_X_FINE_GAIN, gain)
}

// Y_FINE_GAIN register

pub fn get_y_fine_gain(&self) -> Result<u8> {
    i2c::read_byte(self.dev_address, RA_Y_FINE_GAIN)
}

pub fn set_y_fine_gain(&self, gain: u8) -> Result<()> {
    i2c::write_byte(self.dev_address, RA_Y_FINE_GAIN, gain)
}

// Z_FINE_GAIN register

pub fn get_z_fine_gain(&self) -> Result<u8> {
    i2c::read_byte(self.dev_address, RA_Z_FINE_GAIN)
}

pub fn set_z_fine_gain(&self, gain: u8) -> Result<()> {
    i2c::write_byte(self.dev_address, RA_Z_FINE_GAIN, gain)
}

// XA_OFFS_* registers

pub fn get_x_accel_offset(&self) -> Result<i16> {
    let mut buffer = [0; 2];
    i2c::read_bytes(self.dev_address, RA_XA_OFFS_H, 2, &mut buffer)?;
    Ok(((buffer[0] as i16) << 8) | (buffer[1] as i16))
}

pub fn set_x_accel_offset(&self, offset: u16) -> Result<()> {
    i2c::write_word(self.dev_address, RA_XA_OFFS_H, offset)
}

// YA_OFFS_* register

pub fn get_y_accel_offset(&self) -> Result<i16> {
    let mut buffer = [0; 2];
    i2c::read_bytes(self.dev_address, RA_YA_OFFS_H, 2, &mut buffer)?;
    Ok(((buffer[0] as i16) << 8) | (buffer[1] as i16))
}

pub fn set_y_accel_offset(&self, offset: u16) -> Result<()> {
    i2c::write_word(self.dev_address, RA_YA_OFFS_H, offset)
}

// ZA_OFFS_* register

pub fn get_z_accel_offset(&self) -> Result<i16> {
    let mut buffer = [0; 2];
    i2c::read_bytes(self.dev_address, RA_ZA_OFFS_H, 2, &mut buffer)?;
    Ok(((buffer[0] as i16) << 8) | (buffer[1] as i16))
}

pub fn set_z_accel_offset(&self, offset: u16) -> Result<()> {
    i2c::write_word(self.dev_address, RA_ZA_OFFS_H, offset)
}

// XG_OFFS_USR* registers

pub fn get_x_gyro_offset_user(&self) -> Result<i16> {
    let mut buffer = [0; 2];
    i2c::read_bytes(self.dev_address, RA_XG_OFFS_USRH, 2, &mut buffer)?;
    Ok(((buffer[0] as i16) << 8) | (buffer[1] as i16))
}

pub fn set_x_gyro_offset_user(&self, offset: u16) -> Result<()> {
    i2c::write_word(self.dev_address, RA_XG_OFFS_USRH, offset)
}

// YG_OFFS_USR* register

pub fn get_y_gyro_offset_user(&self) -> Result<i16> {
    let mut buffer = [0; 2];
    i2c::read_bytes(self.dev_address, RA_YG_OFFS_USRH, 2, &mut buffer)?;
    Ok(((buffer[0] as i16) << 8) | (buffer[1] as i16))
}

pub fn set_y_gyro_offset_user(&self, offset: u16) -> Result<()> {
    i2c::write_word(self.dev_address, RA_YG_OFFS_USRH, offset)
}

// ZG_OFFS_USR* register

pub fn get_z_gyro_offset_user(&self) -> Result<i16> {
    let mut buffer = [0; 2];
    i2c::read_bytes(self.dev_address, RA_ZG_OFFS_USRH, 2, &mut buffer)?;
    Ok(((buffer[0] as i16) << 8) | (buffer[1] as i16))
}

pub fn set_z_gyro_offset_user(&self, offset: u16) -> Result<()> {
    i2c::write_word(self.dev_address, RA_ZG_OFFS_USRH, offset)
}

// INT_ENABLE register (DMP functions)

pub fn get_int_pll_ready_enabled(&self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_INT_ENABLE, INTERRUPT_PLL_RDY_INT_BIT)
}

pub fn set_int_pll_ready_enabled(&self, enabled: u8) -> Result<()> {
    i2c::write_bit(self.dev_address, RA_INT_ENABLE, INTERRUPT_PLL_RDY_INT_BIT, enabled)
}

pub fn get_int_dmp_enabled(&self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_INT_ENABLE, INTERRUPT_DMP_INT_BIT)
}

pub fn set_int_dmp_enabled(&self, enabled: u8) -> Result<()> {
    i2c::write_bit(self.dev_address, RA_INT_ENABLE, INTERRUPT_DMP_INT_BIT, enabled)
}

// DMP_INT_STATUS

pub fn get_dmp_int5_status(&self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_DMP_INT_STATUS, DMPINT_5_BIT)
}

pub fn get_dmp_int4_status(&self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_DMP_INT_STATUS, DMPINT_4_BIT)
}

pub fn get_dmp_int3_status(&self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_DMP_INT_STATUS, DMPINT_3_BIT)
}

pub fn get_dmp_int2_status(&self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_DMP_INT_STATUS, DMPINT_2_BIT)
}

pub fn get_dmp_int1_status(&self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_DMP_INT_STATUS, DMPINT_1_BIT)
}

pub fn get_dmp_int0_status(&self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_DMP_INT_STATUS, DMPINT_0_BIT)
}

// INT_STATUS register (DMP functions)

pub fn get_int_pll_ready_status(&self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_INT_STATUS, INTERRUPT_PLL_RDY_INT_BIT)
}

pub fn get_int_dmp_status(&self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_INT_STATUS, INTERRUPT_DMP_INT_BIT)
}

// USER_CTRL register (DMP functions)

pub fn get_dmp_enabled(&self) -> Result<u8> {
    i2c::read_bit(self.dev_address, RA_USER_CTRL, USERCTRL_DMP_EN_BIT)
}

pub fn set_dmp_enabled(&self, enabled: u8) -> Result<()> {
    i2c::write_bit(self.dev_address, RA_USER_CTRL, USERCTRL_DMP_EN_BIT, enabled)
}

pub fn reset_dmp(&self) -> Result<()> {
    i2c::write_bit(self.dev_address, RA_USER_CTRL, USERCTRL_DMP_RESET_BIT, 1)
}

// BANK_SEL register

pub fn set_memory_bank(&self, bank: u8, prefetch_enabled: bool, user_bank: bool) -> Result<()> {
    let mut bank_value = bank & 0x1F;
    if user_bank { bank_value |= 0x20; }
    if prefetch_enabled { bank_value |= 0x40; }
    i2c::write_byte(self.dev_address, RA_BANK_SEL, bank_value)
}

// MEM_START_ADDR register

pub fn set_memory_start_address(&self, address: u8) -> Result<()> {
    i2c::write_byte(self.dev_address, RA_MEM_START_ADDR, address)
}

// MEM_R_W register

pub fn read_memory_byte(&self) -> Result<u8> {
    i2c::read_byte(self.dev_address, RA_MEM_R_W)
}

pub fn write_memory_byte(&self, data: u8) -> Result<()> {
    i2c::write_byte(self.dev_address, RA_MEM_R_W, data)
}

pub fn read_memory_block(&self, data: &mut [u8], data_size: u16, mut bank: u8, mut address: u8) -> Result<()> {
    self.set_memory_bank(bank, false, false);
    self.set_memory_start_address(address);
    let mut chunk_size : u8 = 0;
    let mut i : u16 = 0;
    while i < data_size {
        // determine correct chunk size according to bank position and data size
        chunk_size = DMP_MEMORY_CHUNK_SIZE;

        // make sure we don't go past the data size
        if (i + chunk_size as u16 > data_size) {
            chunk_size = (data_size - i) as u8; 
        }  

        // make sure this chunk doesn't go past the bank boundary (256 bytes)
        // This ensures that wrapping_add will only wrap to exactly 0
        if (chunk_size as u16 > 256 - address as u16) {
            chunk_size = ((256 - address as u16) & 0xFF).try_into().unwrap();
        }

        // read the chunk of data as specified
        i2c::read_bytes(self.dev_address, RA_MEM_R_W, chunk_size, &mut data[i as usize .. (i as usize + chunk_size as usize)])?;

        // increase byte index by [chunk_size]
        i += chunk_size as u16;

        // automatically wraps to 0 at 256
        address = address.wrapping_add(chunk_size);

        // if we aren't done, update bank (if necessary) and address
        if i < data_size {
            if address == 0 {
                bank += 1;
            }
            self.set_memory_bank(bank, false, false);
            self.set_memory_start_address(address);
        }
    }
    Ok(())
}

// Omit use_prog_mem as memory constraints in linux environments are not often as tight
pub fn write_memory_block(&self, data: &[u8], data_size: u16, mut bank: u8, mut address: u8, verify: bool) -> Result<()> {
    self.set_memory_bank(bank, false, false);
    self.set_memory_start_address(address);
    let mut verify_buffer = [0u8; DMP_MEMORY_CHUNK_SIZE as usize];
    let mut prog_buffer = [0u8; DMP_MEMORY_CHUNK_SIZE as usize];
    let mut chunk_size : u8 = 0;
    let mut i : u16 = 0;
    while i < data_size {
        // determine correct chunk size according to bank position and data size
        chunk_size = DMP_MEMORY_CHUNK_SIZE;

        // make sure we don't go past the data size
        if (i + chunk_size as u16 > data_size) {
            chunk_size = (data_size - i) as u8; 
        }  

        // make sure this chunk doesn't go past the bank boundary (256 bytes)
        // This ensures that wrapping_add will only wrap to exactly 0
        if (chunk_size as u16 > 256 - address as u16) {
            chunk_size = ((256 - address as u16) & 0xFF).try_into().unwrap();
        }

        // write the chunk of data as specified
        i2c::write_bytes(self.dev_address, RA_MEM_R_W, chunk_size, &data[i as usize .. (i as usize + chunk_size as usize)])?;

        if (verify) {
            self.set_memory_bank(bank, false, false);
            self.set_memory_start_address(address);
            // read the chunk of data as specified
            i2c::read_bytes(self.dev_address, RA_MEM_R_W, chunk_size, &mut verify_buffer[i as usize .. (i as usize + chunk_size as usize)])?;
            if (data[i as usize .. (i as usize + chunk_size as usize)] == verify_buffer[i as usize .. (i as usize + chunk_size as usize)]) {
                return Err(anyhow!("Write Verify Failed")) // uh oh.
            }

        }

        // increase byte index by [chunk_size]
        i += chunk_size as u16;

        // automatically wraps to 0 at 256
        address = address.wrapping_add(chunk_size);

        // if we aren't done, update bank (if necessary) and address
        if i < data_size {
            if address == 0 {
                bank += 1;
            }
            self.set_memory_bank(bank, false, false);
            self.set_memory_start_address(address);
        }
    }
    Ok(())
}

pub fn write_dmp_configuration_set(&self, data: &[u8], data_size: u16) -> Result<()> {
    let mut success : u8 = 0;
    let mut special : u8 = 0;

    let mut i : u16 = 0;
    let mut bank : u8 = 0;
    let mut offset : u8 = 0;
    let mut length : u8 = 0;

    // config set data is a long string of blocks with the following structure:
    // [bank] [offset] [length] [byte[0], byte[1], ..., byte[length]]
    // write data or perform special action
    while i < data_size {
        bank = data[i as usize];
        i += 1;
        offset = data[i as usize];
        i += 1;
        length = data[i as usize];
    }

    // write data or perform special action
    if length > 0 {
       self.write_memory_block(&data[i as usize .. (i as usize + length as usize)], length as u16, bank, offset, true)?;
       i += length as u16;
    }
    else {
        // special instruction
        // NOTE: this kind of behavior (what and when to do certain things)
        // is totally undocumented. This code is in here based on observed
        // behavior only, and exactly why (or even whether) it has to be here
        // is anybody's guess for now.

        special = data[i as usize];
        i += 1;
        match special {
            0x01 => {
                // enable DMP-related interrupts
                //setIntZeroMotionEnabled(true);
                //setIntFIFOBufferOverflowEnabled(true);
                //setIntDMPEnabled(true);
                i2c::write_byte(self.dev_address, RA_INT_ENABLE, 0x32)?;  // simgle operation
            },
            _ => {
                return Err(anyhow!("Unknown Special Command"));
            }
        }
    }

    Ok(())
}

// DMP_CFG_1 register

pub fn get_dmp_config1(&self) -> Result<u8> {
    i2c::read_byte(self.dev_address, RA_DMP_CFG_1)
}

pub fn set_dmp_config1(&self, config: u8) -> Result<()> {
    i2c::write_byte(self.dev_address, RA_DMP_CFG_1, config)
}

// DMP_CFG_2 register

pub fn get_dmp_config2(&self) -> Result<u8> {
    i2c::read_byte(self.dev_address, RA_DMP_CFG_2)
}

pub fn set_dmp_config2(&self, config: u8) -> Result<()> {
    i2c::write_byte(self.dev_address, RA_DMP_CFG_2, config)
}




}

