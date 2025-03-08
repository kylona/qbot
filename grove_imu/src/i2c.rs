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
