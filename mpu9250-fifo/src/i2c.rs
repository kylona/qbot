extern crate i2c_linux;
extern crate log;

use i2c_linux::I2c;
use anyhow::Result;
use log::debug;

const I2C_BUS_PATH : &str = "/dev/i2c-1";
pub fn read_byte(dev_addr : u16, reg_addr : u8) -> Result<u8> {
    let mut i2c = I2c::from_path(I2C_BUS_PATH)?;
    i2c.smbus_set_slave_address(dev_addr, false)?;
    let data = i2c.smbus_read_byte_data(reg_addr)?;
    debug!("Read I2C data: {}", data);
    return Ok(data);
}

pub fn write_byte(dev_addr : u16, reg_addr : u8, data : u8) -> Result<()> {
    let mut i2c = I2c::from_path(I2C_BUS_PATH)?;
    i2c.smbus_set_slave_address(dev_addr, false)?;
    i2c.smbus_write_byte_data(reg_addr, data)?;
    debug!("WROTE I2C data: {}", data);
    return Ok(());
}

pub fn read_bytes(dev_addr : u16, reg_addr : u8, length: u8, data : &mut [u8]) -> Result<u8> {
    let mut i2c = I2c::from_path(I2C_BUS_PATH)?;
    i2c.smbus_set_slave_address(dev_addr, false)?;
    let mut bytes_read = 0;
    for offset in 0..length {
        data[offset as usize] = i2c.smbus_read_byte_data(reg_addr+offset)?;
        bytes_read += 1;
    }
    return Ok(bytes_read);
}

pub fn write_bytes(dev_addr : u16, reg_addr : u8, length : u8, data : &[u8]) -> Result<u8> {
    let mut i2c = I2c::from_path(I2C_BUS_PATH)?;
    i2c.smbus_set_slave_address(dev_addr, false)?;
    let mut bytes_written = 0;
    for offset in 0..length {
        i2c.smbus_write_byte_data(reg_addr+offset, data[offset as usize])?;
        bytes_written += 1;
    }
    return Ok(bytes_written);
}

#[allow(unused)]
pub fn read_word(dev_addr : u16, reg_addr : u8) -> Result<u16> {
    let mut i2c = I2c::from_path(I2C_BUS_PATH)?;
    i2c.smbus_set_slave_address(dev_addr, false)?;
    let data = i2c.smbus_read_word_data(reg_addr)?;
    debug!("Read I2C data: {}", data);
    return Ok(data);
}

pub fn write_word(dev_addr : u16, reg_addr : u8, data : u16) -> Result<()> {
    let mut i2c = I2c::from_path(I2C_BUS_PATH)?;
    i2c.smbus_set_slave_address(dev_addr, false)?;
    i2c.smbus_write_word_data(reg_addr, data)?;
    debug!("WROTE I2C data: {}", data);
    return Ok(());
}

#[allow(unused)]
pub fn read_words(dev_addr : u16, reg_addr : u8, length: u8, data : &mut [u16]) -> Result<u16> {
    let mut i2c = I2c::from_path(I2C_BUS_PATH)?;
    i2c.smbus_set_slave_address(dev_addr, false)?;
    let mut words_read = 0;
    for offset in 0..length {
        data[offset as usize] = i2c.smbus_read_word_data(reg_addr+offset)?;
        words_read += 1;
    }
    return Ok(words_read);
}

#[allow(unused)]
pub fn write_words(dev_addr : u16, reg_addr : u8, length : u8, data : &[u16]) -> Result<u16> {
    let mut i2c = I2c::from_path(I2C_BUS_PATH)?;
    i2c.smbus_set_slave_address(dev_addr, false)?;
    let mut words_written = 0;
    for offset in 0..length {
        i2c.smbus_write_word_data(reg_addr+offset, data[offset as usize])?;
        words_written += 1;
    }
    return Ok(words_written);
}


pub fn read_bits(dev_addr : u16, reg_addr : u8, bit_start : u8, length : u8) -> Result<u8> {
    // 01101001 read byte
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted
    let mut b = read_byte(dev_addr, reg_addr)?;
    let mask : u16 = (((1 << length as u16)) - 1) << (bit_start + 1 - length);
    b &= mask as u8;
    b >>= bit_start - length + 1;
    return Ok(b);
}

pub fn read_bit(dev_addr : u16, reg_addr : u8, bit_start : u8) -> Result<u8> {
    return read_bits(dev_addr, reg_addr, bit_start, 1);
}

pub fn write_bits(dev_addr : u16, reg_addr : u8, bit_start : u8, length : u8, mut data : u8) -> Result<()> {
    let mut b = read_byte(dev_addr, reg_addr)?;
    debug!("bit_start: {}", bit_start);
    debug!("length: {}", length);
    let mask : u16 = (((1 << length as u16)) - 1) << (bit_start + 1 - length);
    data <<= bit_start + 1 - length;
    data &= mask as u8;
    b &= !mask as u8;
    b |= data;
    return write_byte(dev_addr, reg_addr, b);
}

pub fn write_bit(dev_addr : u16, reg_addr : u8, bit_start : u8, data : u8) -> Result<()> {
    return write_bits(dev_addr, reg_addr, bit_start, 1, data)
}

