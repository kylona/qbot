mod mpu9250;
pub use crate::mpu9250::MPU9250;
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
        assert!(mpu9250.initialize().is_ok(), "Initialize failed");
        //mpu9250.set_fifo_enabled(false).expect("Failed to disable fifo");
        //mpu9250.reset_fifo().expect("Failed to reset fifo");
        mpu9250.set_accel_fifo_enabled(true).expect("Failed to enabled accel for fifo");
        assert!(mpu9250.get_accel_fifo_enabled().unwrap() == 1u8, "Fifo accel enable didn't stick?");
        mpu9250.set_x_gyro_fifo_enabled(true).expect("Failed to enable x gyro for fifo");
        mpu9250.set_y_gyro_fifo_enabled(true).expect("Failed to enable y gyro for fifo");
        mpu9250.set_z_gyro_fifo_enabled(true).expect("Failed to enable x gyro for fifo");
        let rate = mpu9250.get_rate().unwrap();
        println!("RATE: {}", rate);
        mpu9250.set_fifo_enabled(true).expect("Failed to enable fifo");
        std::thread::sleep(std::time::Duration::from_millis(100));
        let fifo_count = mpu9250.get_fifo_count().unwrap();
        assert!(fifo_count > 0u16, "FIFO empty after start");
        println!("FIFO Count: {}", fifo_count);
        mpu9250.set_fifo_enabled(false).expect("Failed to disable fifo");
        let mut data = [0u8; 512];
        let data_count = mpu9250.get_fifo_data(&mut data).expect("Fifo block read failed");
        for i in 0..data_count {
            println!("FIFO DATA: {}", data[i]);
        }
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
