mod mpu9250;
pub use crate::mpu9250::MPU9250;
pub use crate::mpu9250::AccelerometerData;
pub use crate::mpu9250::GyroscopeData;
pub use crate::mpu9250::MagnetometerData;
mod i2c;
pub use crate::i2c::read_byte;


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_get_device_id() {
        let mpu9250 = MPU9250::new(0x68);
        assert_eq!(mpu9250.get_device_id().unwrap(), 0x71, "Device Id should be 0x71")
    }

    #[test]
    fn test_get_motion_6() {
        let mut mpu9250 = MPU9250::new(0x68);
        assert!(mpu9250.initialize().is_ok(), "Initialize failed");
        assert!(mpu9250.get_motion_6().is_ok(), "Get motion 6 failed")
    }

    #[test]
    fn test_get_motion_9() {
        let mut mpu9250 = MPU9250::new(0x68);
        assert!(mpu9250.initialize().is_ok(), "Initialize failed");
        for _ in 0..100 {
            let (accel_data, gyro_data, mag_data) = mpu9250.get_motion_9().unwrap();
            println!("ALL MOTION DATA:");
            println!("Acc\tx:{}\ty:{}\tz:{}", accel_data.x, accel_data.y, accel_data.z);
            println!("Gyr:\tx:{}\ty:{}\tz:{}\n", gyro_data.x, gyro_data.y, gyro_data.z);
            println!("Mag:\tx:{}\ty:{}\tz:{}\n", mag_data.x, mag_data.y, mag_data.z);
            std::thread::sleep(std::time::Duration::from_millis(100));
        }
        assert!(mpu9250.get_motion_9().is_ok(), "Get motion 6 failed")
    }

    #[test]
    fn test_fifo_flush() {
        let mut mpu9250 = MPU9250::new(0x68);
        assert!(mpu9250.initialize().is_ok(), "Initialize failed");
        mpu9250.flush_fifo().expect("Flush fifo failed");
        let fifo_count = mpu9250.get_fifo_count().expect("Get fifo count failed");
        assert_eq!(fifo_count, 0, "FIFO not empty after flush");
    }

    #[test]
    fn test_fifo_motion_6() {
        let mut mpu9250 = MPU9250::new(0x68);
        let mut data = [0u8; 512];
        let mut accel_data = [AccelerometerData { x: 0, y : 0, z: 0}; 85];
        let mut gyro_data = [GyroscopeData { x: 0, y : 0, z: 0}; 85];

        assert!(mpu9250.initialize().is_ok(), "Initialize failed");
        mpu9250.set_rate(200).expect("Set rate failed");
        let rate = mpu9250.get_rate().unwrap();
        println!("RATE: {}", rate);
        mpu9250.set_fifo_enabled(true).expect("Failed to enable fifo");
        mpu9250.flush_fifo().expect("Flush fifo failed");
        let fifo_enabled_flags : u8 = 0b01111000;
        mpu9250.set_fifo_enabled_flags(fifo_enabled_flags).expect("Setting fifo enable flags failed");
        let data_count_1 = mpu9250.get_fifo_data(&mut data).expect("Fifo block read failed");
        let data_count_2 = mpu9250.get_fifo_data(&mut data).expect("Fifo block read failed");
        mpu9250.set_fifo_enabled_flags(0x0).expect("Reseting fifo enable flags failed");
        let remaining_data_count = mpu9250.parse_fifo_data(&data, &mut accel_data, &mut gyro_data, data_count_2, fifo_enabled_flags).expect("Parse data failed");
        println!("DATA COUNT 1: {}", data_count_1);
        println!("DATA COUNT 2: {}", data_count_2);
        println!("FIFO DATA: {:?}", data);
        println!("PARSED ACCEL DATA: {:?}", accel_data);
        println!("PARSED GYRO DATA: {:?}", gyro_data);
        let (accel_data, gyro_data) = mpu9250.get_motion_6().expect("Get motion 6 failed");
        println!("ALL MOTION DATA:");
        println!("Acc\tx:{}\ty:{}\tz:{}", accel_data.x, accel_data.y, accel_data.z);
        println!("Gyr:\tx:{}\ty:{}\tz:{}\n", gyro_data.x, gyro_data.y, gyro_data.z);
    }

    #[test]
    fn test_read_accelerometer() {
        let mut mpu9250 = MPU9250::new(0x68);
        assert!(mpu9250.initialize().is_ok(), "Initialize failed");
        assert!(mpu9250.get_acceleration_x().is_ok(), "Read accelerometer failed")
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
