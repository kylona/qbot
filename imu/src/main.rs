extern crate mpu9250_fifo;
use mpu9250_fifo::MPU9250;
use ahrs::{Ahrs, Madgwick};
use nalgebra::Vector3;
use std::f64;

const DATA_RATE = 500.0

fn main() {
    // Initialize filter with default values
    let mut ahrs = Madgwick::new(1.0/DATA_RATE, 0.1);

    let mut mpu9250 = MPU9250::default();
    mpu9250.initialize();
    let mut accel_meas = [AccelerometerMeasurement { x: 0.0, y : 0.0, z: 0.0}; 50];
    let mut gyro_meas = [GyroscopeMeasurement { x: 0.0, y : 0.0, z: 0.0}; 50];
    let mut mag_meas = [MagnetometerMeasurementMeasurement { x: 0.0, y : 0.0, z: 0.0}; 50];
    let fifo_enabled_flags : u8 = 0b01111000;
    mpu9250.set_fifo_enabled(true).expect("Fifo enable failed");
    mpu9250.set_fifo_rate(u16::from(DATA_RATE)).expect("Set fifo rate failed");
    mpu9250.flush_fifo().expect("Flush fifo failed");
    let fifo_enabled_flags : u8 = 0b01111000;
    mpu9250.set_fifo_enabled_flags(fifo_enabled_flags).expect("Setting fifo enable flags failed");
    loop {
        let data_count = mpu9250.get_fifo_measurements(accel_meas, gyro_meas, 0)
        let (accel_meas, gyro_meas, mag_meas) = mpu9250.measure_motion_9().expect("Could not connect to mpu9250");
        println!("Measured Data:");
        println!("Accel: {:?}", accel_meas);
        println!("Gyro: {:?}", gyro_meas);
        println!("Mag: {:?}", mag_meas);

        for i in 0..data_count {
            // Obtain sensor values from a source
            let gyroscope = Vector3::new(gyro_meas[i].x as f64, gyro_meas[i].y as f64, gyro_meas[i].z as f64);
            let accelerometer = Vector3::new(accel_meas[i].x as f64, accel_meas[i].y as f64, accel_meas[i].z as f64);
            // let magnetometer = Vector3::new(mag_meas.x as f64, mag_meas.y as f64, mag_meas.z as f64);

            // Run inputs through AHRS filter (gyroscope must be radians/s)
            let quat = ahrs
                .update_imu(
                    &(gyroscope * (f64::consts::PI / 180.0)),
                    &accelerometer,
                    //&magnetometer,
                )
                .unwrap();
            let (roll, pitch, yaw) = quat.euler_angles();
        }
        // Do something with the updated state quaternion
        println!("pitch={}, roll={}, yaw={}", pitch * 180.0 /f64::consts::PI, roll * 180.0 /f64::consts::PI, yaw * 180.0 /f64::consts::PI);
    }
}
