extern crate mpu9250_fifo;
use mpu9250_fifo::mpu9250::{MPU9250};
use mpu9250_fifo::mpu9250::{AccelerometerMeasurement, GyroscopeMeasurement, MagnetometerMeasurement};
use ahrs::{Ahrs, Madgwick};
use nalgebra::Vector3;
use std::f64;

const DATA_RATE : u16 = 500;

fn main() {
    // Initialize filter with default values
    let mut ahrs = Madgwick::new(1.0/f64::from(DATA_RATE), 0.1);

    let mut mpu9250 = MPU9250::default();
    mpu9250.initialize().expect("Failed to initialize mpu9250");
    let mut accel_meas = [AccelerometerMeasurement { x: 0.0, y : 0.0, z: 0.0}; 100];
    let mut gyro_meas = [GyroscopeMeasurement { x: 0.0, y : 0.0, z: 0.0}; 100];
    let mut mag_meas = [MagnetometerMeasurement { x: 0.0, y : 0.0, z: 0.0}; 100];
    mpu9250.set_fifo_enabled(true).expect("Fifo enable failed");
    mpu9250.set_fifo_rate(u16::from(DATA_RATE)).expect("Set fifo rate failed");
    let fifo_enabled_flags : u8 = 0b01111000;
    mpu9250.set_fifo_enabled_flags(fifo_enabled_flags).expect("Setting fifo enable flags failed");
    mpu9250.flush_fifo().expect("Flush fifo failed");
    loop {
        let data_count = mpu9250.get_fifo_measurements(&mut accel_meas, &mut gyro_meas, &mut mag_meas, fifo_enabled_flags).expect("Fetch fifo data failed");
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
            let magnetometer = Vector3::new(mag_meas[i].x as f64, mag_meas[i].y as f64, mag_meas[i].z as f64);
            println!("Mag: {:?}", mag_meas[i]);

            // Run inputs through AHRS filter (gyroscope must be radians/s)
            quat = match ahrs.update(
                &(gyroscope * (f64::consts::PI / 180.0)),
                &accelerometer,
                &magnetometer,
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
        println!("pitch={}, roll={}, yaw={}", pitch * 180.0 /f64::consts::PI, roll * 180.0 /f64::consts::PI, yaw * 180.0 /f64::consts::PI);
    }
}
