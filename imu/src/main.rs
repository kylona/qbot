extern crate mpu9250_fifo;
use mpu9250_fifo::MPU9250;
use ahrs::{Ahrs, Madgwick};
use nalgebra::Vector3;
use std::f64;

fn main() {
    // Initialize filter with default values
    let mut ahrs = Madgwick::new(0.01, 0.01);

    let mut mpu9250 = MPU9250::default();
    loop {
        let (accel_meas, gyro_meas, mag_meas) = mpu9250.measure_motion_9().expect("Could not connect to mpu9250");
        println!("Measured Data:");
        println!("Accel: {:?}", accel_meas);
        println!("Gyro: {:?}", gyro_meas);
        println!("Mag: {:?}", mag_meas);

        // Obtain sensor values from a source
        let gyroscope = Vector3::new(gyro_meas.x as f64, gyro_meas.y as f64, gyro_meas.z as f64);
        let accelerometer = Vector3::new(accel_meas.x as f64, accel_meas.y as f64, accel_meas.z as f64);
        let magnetometer = Vector3::new(mag_meas.x as f64, mag_meas.y as f64, mag_meas.z as f64);

        // Run inputs through AHRS filter (gyroscope must be radians/s)
        let quat = ahrs
            .update_imu(
                &(gyroscope * (f64::consts::PI / 180.0)),
                &accelerometer,
                //&magnetometer,
            )
            .unwrap();
        let (roll, pitch, yaw) = quat.euler_angles();

        // Do something with the updated state quaternion
        println!("pitch={}, roll={}, yaw={}", pitch * 180.0 /f64::consts::PI, roll * 180.0 /f64::consts::PI, yaw * 180.0 /f64::consts::PI);
        std::thread::sleep(std::time::Duration::from_millis(9));
    }
}
