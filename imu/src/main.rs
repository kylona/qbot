extern crate mpu9250_fifo;
use mpu9250_fifo::mpu9250::{MPU9250};
use mpu9250_fifo::mpu9250::{AccelerometerMeasurement, GyroscopeMeasurement, MagnetometerMeasurement};
use ahrs::{Ahrs, Madgwick};
use nalgebra::Vector3;
use std::f64;
use std::io::Write;
use std::io::stdout;

fn main() {
    let mut accel_meas = [AccelerometerMeasurement { x: 0.0, y : 0.0, z: 0.0}; 100];
    let mut gyro_meas = [GyroscopeMeasurement { x: 0.0, y : 0.0, z: 0.0}; 100];
    let mut mag_meas = [MagnetometerMeasurement { x: 0.0, y : 0.0, z: 0.0}; 100];

    // Initialize filter with default values
    let mut ahrs = Madgwick::new(1.0/f64::from(500), 0.1);
    let mut mpu9250 = MPU9250::new(
        None,
        None,
        None,
        None,
        Some(true),
        Some(true),
        Some(false),
    );
    mpu9250.initialize().expect("Failed to initialize mpu9250");
    mpu9250.start_fifo().expect("Failed to start FIFO");
    loop {
        let data_count = match mpu9250.get_fifo_measurements(&mut accel_meas, &mut gyro_meas, &mut mag_meas) {
            Ok(count) => count,
            Err(_) => {
                println!("FIFO Overflow detected");
                mpu9250.flush_fifo().expect("FIFO Flush after overflow failed");
                continue;
            }
        };
        //let (accel_meas_datum, gyro_meas_datum, mag_meas_datum) = mpu9250.measure_motion_9().expect("Could not connect to mpu9250");
        //println!("Measured Data:");
        //println!("Accel: {:?}", accel_meas_datum);
        //println!("Gyro: {:?}", gyro_meas_datum);
        //println!("Mag: {:?}", mag_meas_datum);

        let mut quat = ahrs.quat.clone();
        print!("DATA COUNT: {} ", data_count);
        for i in 0..data_count {
            // Obtain sensor values from a source
            let gyroscope = Vector3::new(gyro_meas[i].x as f64, gyro_meas[i].y as f64, gyro_meas[i].z as f64);
            let accelerometer = Vector3::new(accel_meas[i].x as f64, accel_meas[i].y as f64, accel_meas[i].z as f64);
            //let magnetometer = Vector3::new(mag_meas[i].x as f64, mag_meas[i].y as f64, mag_meas[i].z as f64);

            // Run inputs through AHRS filter (gyroscope must be radians/s)
            quat = match ahrs.update_imu(
                &(gyroscope * (f64::consts::PI / 180.0)),
                &accelerometer,
                //&magnetometer,
            ) {
                Ok(val) => *val,
                Err(_) => {
                    continue;
                }
            };
        }
        std::thread::sleep(std::time::Duration::from_millis(10));
        let (roll, pitch, yaw) = quat.euler_angles();
        // Do something with the updated state quaternion
        print!("pitch={:0.5}, roll={:0.5}, yaw={:0.5}\r", pitch * 180.0 /f64::consts::PI, roll * 180.0 /f64::consts::PI, yaw * 180.0 /f64::consts::PI);
        stdout().flush().expect("Flush std out failed");
    }
}
