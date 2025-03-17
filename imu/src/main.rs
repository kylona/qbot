extern crate mpu9250_fifo;
use mpu9250_fifo::MPU9250;
use ahrs::{Ahrs, Madgwick};
use nalgebra::Vector3;
use std::f64;

fn main() {
    // Initialize filter with default values
    let mut ahrs = Madgwick::default();

    let mut mpu9250 = MPU9250::new(0x68);
    let (accel_data, gyro_data, mag_data) = mpu9250.get_motion_9().expect("Could not connect to mpu9250");
    // Obtain sensor values from a source
    let gyroscope = Vector3::new(gyro_data.x, gyro_data.y, gyro_data.z);
    let accelerometer = Vector3::new(accel_data.x, accel_data.y, accel_data.z);
    let magnetometer = Vector3::new(mag_data.x, mag_data.y, mag_data.z);

    // Run inputs through AHRS filter (gyroscope must be radians/s)
    let quat = ahrs
        .update(
            &(gyroscope * (f64::consts::PI / 180.0)),
            &accelerometer,
            &magnetometer,
        )
        .unwrap();
    let (roll, pitch, yaw) = quat.euler_angles();

    // Do something with the updated state quaternion
    println!("pitch={}, roll={}, yaw={}", pitch, roll, yaw);
}
