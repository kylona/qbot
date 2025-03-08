mod mpu9250;
pub use crate::mpu9250::MPU9250;
pub use crate::mpu9250::read_x_accelerometer;
mod i2c;
pub use crate::i2c::read_byte;


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_get_device_id() {
        let mut mpu9250 = MPU9250::new(0x68);
        assert_eq!(mpu9250.get_device_id().unwrap(), 0x71, "Device Id should be 0x71")
    }

    #[test]
    fn test_read_accelerometer() {
        assert!(read_x_accelerometer().is_ok(), "Read accelerometer failed")
    }

    #[test]
    fn test_read_byte() {
        assert!(read_byte(0x68, 0x3F).is_ok(), "Read byte failed")
    }

    #[test]
    fn test_initialize() {
        let mut mpu9250 = MPU9250::new(0x68);
        assert!(mpu9250.initialize().is_ok(), "Initialize failed")
    }
}
