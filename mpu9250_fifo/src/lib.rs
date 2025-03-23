pub mod mpu9250;
pub mod i2c;
pub mod calibrate;
pub use crate::mpu9250::{MPU9250, AccelerometerData, AccelerometerMeasurement, GyroscopeData, GyroscopeMeasurement, MagnetometerData, MagnetometerMeasurement, sample_rate};
pub use crate::i2c::read_byte;


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_get_device_id() {
        let mpu9250 = MPU9250::default();
        assert_eq!(mpu9250.get_device_id().unwrap(), 0x71, "Device Id should be 0x71")
    }

    #[test]
    fn test_get_motion_6() {
        let mut mpu9250 = MPU9250::default();
        assert!(mpu9250.initialize().is_ok(), "Initialize failed");
        assert!(mpu9250.get_motion_6().is_ok(), "Get motion 6 failed")
    }
    #[test]
    fn test_measure_motion_9() {
        let mut mpu9250 = MPU9250::default();
        assert!(mpu9250.initialize().is_ok(), "Initialize failed");
        let (accel_meas, gyro_meas, mag_meas) = mpu9250.measure_motion_9().unwrap();
        println!("Measured Data:");
        println!("Accel: {:?}", accel_meas);
        println!("Gyro: {:?}", gyro_meas);
        println!("Mag: {:?}", mag_meas);
    }

    #[test]
    fn test_get_motion_9() {
        let mut mpu9250 = MPU9250::default();
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
        let mut mpu9250 = MPU9250::default();
        assert!(mpu9250.initialize().is_ok(), "Initialize failed");
        mpu9250.flush_fifo().expect("Flush fifo failed");
        let fifo_count = mpu9250.get_fifo_count().expect("Get fifo count failed");
        assert_eq!(fifo_count, 0, "FIFO not empty after flush");
    }

    #[test]
    fn test_fifo_motion_6() {
        let mut mpu9250 = MPU9250::default();
        let mut data = [0u8; 512];
        let mut accel_data = [AccelerometerData { x: 0, y : 0, z: 0}; 86];
        let mut gyro_data = [GyroscopeData { x: 0, y : 0, z: 0}; 50];
        let mut mag_data = [MagnetometerData { x: 0, y : 0, z: 0}; 50];

        assert!(mpu9250.initialize().is_ok(), "Initialize failed");
        for _ in 0..100 {
            assert!(mpu9250.start_fifo().is_ok(), "Start FIFO failed");
            std::thread::sleep(std::time::Duration::from_millis(500));
            assert!(mpu9250.stop_fifo().is_ok(), "Start FIFO failed");
            let data_count = mpu9250.get_fifo_data(&mut data).expect("Fifo block read failed");
            let frame_count = mpu9250.parse_fifo_data(&data, &mut accel_data, &mut gyro_data, &mut mag_data, data_count).expect("Parse data failed");
            println!("DATA COUNT: {}", data_count);
            println!("FRAME COUNT: {}", frame_count);
            println!("PARSED ACCEL DATA: {:?}", accel_data[0]);
            println!("PARSED GYRO DATA: {:?}", gyro_data[0]);
        }
        mpu9250.set_fifo_enabled_flags(0x0).expect("Reseting fifo enable flags failed");
    }

    #[test]
    fn test_fifo_motion_9() {
        let mut mpu9250 = MPU9250::new(
            None,
            None,
            None,
            Some(sample_rate::FREQUENCY_100_HZ),
            Some(true),
            Some(true),
            Some(true),
        );
        let mut data = [0u8; 512];
        let mut accel_data = [AccelerometerData { x: 0, y : 0, z: 0}; 86];
        let mut gyro_data = [GyroscopeData { x: 0, y : 0, z: 0}; 50];
        let mut mag_data = [MagnetometerData { x: 0, y : 0, z: 0}; 50];

        assert!(mpu9250.initialize().is_ok(), "Initialize failed");
        for _ in 0..100 {
            assert!(mpu9250.start_fifo().is_ok(), "Start FIFO failed");
            std::thread::sleep(std::time::Duration::from_millis(500));
            assert!(mpu9250.stop_fifo().is_ok(), "Start FIFO failed");
            let data_count = mpu9250.get_fifo_data(&mut data).expect("Fifo block read failed");
            let frame_count = mpu9250.parse_fifo_data(&data, &mut accel_data, &mut gyro_data, &mut mag_data, data_count).expect("Parse data failed");
            println!("DATA COUNT: {}", data_count);
            println!("FRAME COUNT: {}", frame_count);
            println!("PARSED ACCEL DATA: {:?}", accel_data[0]);
            println!("PARSED GYRO DATA: {:?}", gyro_data[0]);
            println!("PARSED GYRO DATA: {:?}", mag_data[0]);
        }
        mpu9250.set_fifo_enabled_flags(0x0).expect("Reseting fifo enable flags failed");
    }

    #[test]
    fn test_fifo_measure_9() {
        let mut accel_meas = [AccelerometerMeasurement { x: 0.0, y : 0.0, z: 0.0}; 100];
        let mut gyro_meas = [GyroscopeMeasurement { x: 0.0, y : 0.0, z: 0.0}; 100];
        let mut mag_meas = [MagnetometerMeasurement { x: 0.0, y : 0.0, z: 0.0}; 100];

        let mut mpu9250 = MPU9250::new(
            None,
            None,
            None,
            Some(sample_rate::FREQUENCY_100_HZ),
            Some(true),
            Some(true),
            Some(true),
        );
        mpu9250.initialize().expect("Failed to initialize mpu9250");
        for _ in 0..5 {
            let frame_count = match mpu9250.get_fifo_measurements(&mut accel_meas, &mut gyro_meas, &mut mag_meas) {
                Ok(count) => count,
                Err(_) => {
                    println!("FIFO Overflow detected");
                    mpu9250.flush_fifo().expect("Flush fifo after overflow failed");
                    continue;
                }
            };
            println!("FRAME COUNT: {}", frame_count);
            for i in 0..frame_count {
                println!("Accel: {:?}", accel_meas[i]);
                println!("Gyro: {:?}", gyro_meas[i]);
                println!("Mag: {:?}", mag_meas[i]);
            }
            std::thread::sleep(std::time::Duration::from_millis(500));
        }
    }

    #[test]
    fn test_fifo_magnetometer() {
        let mut data = [0u8; 512];
        let mut accel_data = [AccelerometerData { x: 0, y : 0, z: 0}; 86];
        let mut gyro_data = [GyroscopeData { x: 0, y : 0, z: 0}; 50];
        let mut mag_data = [MagnetometerData { x: 0, y : 0, z: 0}; 86];

        let mut mpu9250 = MPU9250::new(
            None,
            None,
            None,
            Some(sample_rate::FREQUENCY_100_HZ),
            Some(false),
            Some(false),
            Some(true),
        );
        mpu9250.initialize().expect("Failed to initialize mpu9250");
        for _ in 0..100 {
            assert!(mpu9250.start_fifo().is_ok(), "Start FIFO failed");
            std::thread::sleep(std::time::Duration::from_millis(500));
            assert!(mpu9250.stop_fifo().is_ok(), "Start FIFO failed");
            let data_count = mpu9250.get_fifo_data(&mut data).expect("Fifo block read failed");
            let frame_count = mpu9250.parse_fifo_data(&data, &mut accel_data, &mut gyro_data, &mut mag_data, data_count).expect("Parse data failed");
            println!("DATA COUNT: {}", data_count);
            println!("FRAME_COUNT: {}", frame_count);
            println!("PARSED MAG DATA: {:?}", mag_data[0]);
        }
        mpu9250.set_fifo_enabled_flags(0x0).expect("Reseting fifo enable flags failed");
    }

    #[test]
    fn test_read_accelerometer() {
        let mut mpu9250 = MPU9250::default();
        assert!(mpu9250.initialize().is_ok(), "Initialize failed");
        assert!(mpu9250.get_acceleration_x().is_ok(), "Read accelerometer failed")
    }

    #[test]
    fn test_read_byte() {
        assert!(read_byte(0x68, 0x3F).is_ok(), "Read byte failed")
    }

    #[test]
    fn test_calibrate() {
        let mut mpu9250 = MPU9250::default();
        assert!(mpu9250.initialize().is_ok(), "Initialize failed");
        assert!(mpu9250.calibrate().is_ok(), "Calibrate accelerometer failed")
    }

    #[test]
    fn test_initialize() {
        let mut mpu9250 = MPU9250::default();
        assert!(mpu9250.initialize().is_ok(), "Initialize failed")
    }
}
