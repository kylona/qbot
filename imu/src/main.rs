extern crate mpu9250_fifo;
pub mod simpsons;
use crate::simpsons::SimpsonsIntegral;
use mpu9250_fifo::mpu9250::{MPU9250};
use mpu9250_fifo::mpu9250::{AccelerometerMeasurement, GyroscopeMeasurement, MagnetometerMeasurement};
use ahrs::{Ahrs, Madgwick};
use nalgebra::Vector3;
use std::f64;
use std::io::Write;
use std::io::stdout;

const G_TO_METERS_PER_SEC2 : f32 = 9.80665;

fn main() {
    let mut accel_meas = [AccelerometerMeasurement { x: 0.0, y : 0.0, z: 0.0}; 100];
    let mut gyro_meas = [GyroscopeMeasurement { x: 0.0, y : 0.0, z: 0.0}; 100];
    let mut mag_meas = [MagnetometerMeasurement { x: 0.0, y : 0.0, z: 0.0}; 100];

    // Initialize filter with default values
    let mut ahrs = Madgwick::new(1.0/500.0, 0.1);
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
    let mut earth_frame_accelerometer = Vector3::new(accel_meas[0].x as f64, accel_meas[0].y as f64, accel_meas[0].z as f64);
    let mut velocity_x : f32 = 0.0;
    let mut simpsons_x = SimpsonsIntegral::new(1.0/500.0, 0.0);
    let mut velocity_y : f32 = 0.0;
    let mut simpsons_y = SimpsonsIntegral::new(1.0/500.0, 0.0);
    let mut velocity_z : f32 = 0.0;
    let mut simpsons_z = SimpsonsIntegral::new(1.0/500.0, 0.0);

    let mut pos_x : f32 = 0.0;
    let mut simpsons_pos_x = SimpsonsIntegral::new(1.0/500.0, 0.0);
    let mut pos_y : f32 = 0.0;
    let mut simpsons_pos_y = SimpsonsIntegral::new(1.0/500.0, 0.0);
    let mut pos_z : f32 = 0.0;
    let mut simpsons_pos_z = SimpsonsIntegral::new(1.0/500.0, 0.0);

    let mut loop_count = 0;
    let velocity_decay = 0.01;
    loop {
        loop_count += 1;
        if loop_count % 100 == 0 {
            simpsons_x.reset(0.0);
            simpsons_y.reset(0.0);
            simpsons_z.reset(0.0);
            simpsons_pos_x.reset(0.0);
            simpsons_pos_y.reset(0.0);
            simpsons_pos_z.reset(0.0);
            println!("Reset: \n\n\n\n\n\n\n\n\n");
        }
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
            earth_frame_accelerometer = quat.transform_vector(&accelerometer);
            velocity_x = simpsons_x.update(earth_frame_accelerometer[0] as f32 - (velocity_x.abs() + velocity_decay) * velocity_x.signum());
            velocity_y = simpsons_y.update(earth_frame_accelerometer[1] as f32 - (velocity_y.abs() + velocity_decay) * velocity_y.signum());
            velocity_z = simpsons_z.update(earth_frame_accelerometer[2] as f32 - (velocity_z.abs() + velocity_decay) * velocity_z.signum());
            pos_x = simpsons_pos_x.update(velocity_x * G_TO_METERS_PER_SEC2);
            pos_y = simpsons_pos_y.update(velocity_y * G_TO_METERS_PER_SEC2);
            pos_z = simpsons_pos_z.update(velocity_z * G_TO_METERS_PER_SEC2);
        }

        // std::thread::sleep(std::time::Duration::from_millis(10));
        let (roll, pitch, yaw) = quat.euler_angles();
        // Do something with the updated state quaternion
        print!("DATA COUNT: {}\t\tLoop Count: {}\n ", data_count, loop_count);
        print!("EFA:\t\t {:0.5}\t\t {:0.5}\t\t {:0.5}\n", earth_frame_accelerometer[0], earth_frame_accelerometer[1], earth_frame_accelerometer[2] - 1.0);
        print!("x_vel={:0.5}\t\t y_vel={:0.5}\t\t z_vel={:0.5}\n", velocity_x * G_TO_METERS_PER_SEC2, velocity_y * G_TO_METERS_PER_SEC2, velocity_z * G_TO_METERS_PER_SEC2);
        print!("x_pos={:0.5}\t\t y_pos={:0.5}\t\t z_pos={:0.5}\n", pos_x * G_TO_METERS_PER_SEC2 * 100.0, pos_y * G_TO_METERS_PER_SEC2 * 100.0, pos_z * G_TO_METERS_PER_SEC2 * 100.0);
        print!("pitch={:0.5}\t\t roll={:0.5}\t\t yaw={:0.5}", pitch * 180.0 /f64::consts::PI, roll * 180.0 /f64::consts::PI, yaw * 180.0 /f64::consts::PI);
        print!("\x1b[F\x1b[F\x1b[F\x1b[F");
        stdout().flush().expect("Flush std out failed");
    }
}
