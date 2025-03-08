mod mpu9250;
pub use crate::mpu9250::read_x_accelerometer;
mod i2c;
pub use crate::i2c::read_byte;


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_read_accelerometer() {
        assert!(read_x_accelerometer().is_ok(), "Read accelerometer failed")
    }

    #[test]
    fn test_read_byte() {
        assert!(read_byte(0x68, 0x3F).is_ok(), "Read byte failed")
    }
}
