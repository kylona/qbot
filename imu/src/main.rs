extern crate mpu9250_fifo;
pub mod sync;
use crate::sync::{SyncConnection, Imu, Header, Time, Quaternion, ImuVector3};
use mpu9250_fifo::mpu9250::{MPU9250, sample_rate};
use mpu9250_fifo::mpu9250::{AccelerometerMeasurement, GyroscopeMeasurement, MagnetometerMeasurement};
use ahrs::{Ahrs, Madgwick};
use nalgebra::Vector3;
use std::f64;
use std::time::{Duration, SystemTime};


const G_TO_METERS_PER_SEC2 : f64 = 9.80665;
const SAMPLE_PERIOD : f64 = 1.0/200.0;

fn get_timestamp(start_time : SystemTime, sample_num : u64) -> Time {
    let sample_time = start_time + Duration::from_secs_f64(sample_num as f64 * SAMPLE_PERIOD);
    let stamp_time = sample_time.duration_since(SystemTime::UNIX_EPOCH).unwrap();
    Time {
       sec: stamp_time.as_secs() as i32,
       nsec: stamp_time.subsec_nanos() as i32,
    }
}

#[tokio::main]
async fn main() {
    let mut accel_meas = [AccelerometerMeasurement { x: 0.0, y : 0.0, z: 0.0}; 100];
    let mut gyro_meas = [GyroscopeMeasurement { x: 0.0, y : 0.0, z: 0.0}; 100];
    let mut mag_meas = [MagnetometerMeasurement { x: 0.0, y : 0.0, z: 0.0}; 100];

    // Initialize filter with default values
    let sync_connection = SyncConnection::default().await;
    let mut ahrs = Madgwick::new(SAMPLE_PERIOD, 0.5);
    let mut mpu9250 = MPU9250::new(
        None,
        None,
        None,
        Some(sample_rate::FREQUENCY_200_HZ),
        Some(true),
        Some(true),
        Some(false),
    );

    mpu9250.initialize().expect("Failed to initialize mpu9250");
    mpu9250.start_fifo().expect("Failed to start FIFO");
    let mut fifo_start = SystemTime::now();
    let mut total_num_samples : u64 = 0;
    loop {
        let data_count = match mpu9250.get_fifo_measurements(&mut accel_meas, &mut gyro_meas, &mut mag_meas) {
            Ok(count) => count,
            Err(_) => {
                println!("FIFO Overflow detected");
                mpu9250.flush_fifo().expect("FIFO Flush after overflow failed");
                fifo_start = SystemTime::now();
                continue;
            }
        };

        if data_count > 0 {
            let mut imu_messages : Vec<Imu> = Vec::with_capacity(data_count);
            for i in 0..data_count {
                let timestamp = get_timestamp(fifo_start, total_num_samples);
                total_num_samples += 1;
                // Obtain sensor values from a source
                let gyroscope = Vector3::new(gyro_meas[i].x as f64, gyro_meas[i].y as f64, -gyro_meas[i].z as f64);
                let accelerometer = Vector3::new(accel_meas[i].x as f64, accel_meas[i].y as f64, accel_meas[i].z as f64);
                //let magnetometer = Vector3::new(mag_meas[i].x as f64, mag_meas[i].y as f64, mag_meas[i].z as f64);

                // Run inputs through AHRS filter (gyroscope must be radians/s)
                let quat = match ahrs.update_imu(
                    &(gyroscope * (f64::consts::PI / 180.0)),
                    &accelerometer,
                    //&magnetometer,
                ) {
                    Ok(val) => *val,
                    Err(_) => {
                        continue;
                    }
                };
                let result_quat = quat.inverse();
                imu_messages.push(Imu {
                    header: Header {
                       stamp: timestamp,
                       frame_id: String::from("qbot")
                    },
                    orientation: Quaternion {
                        x: result_quat[0],
                        y: result_quat[1],
                        z: result_quat[2],
                        w: result_quat[3],
                    },
		    orientation_covariance: [
			0.001745329252, 0.0, 0.0,
			0.0, 0.001745329252, 0.0,
			0.0, 0.0, 0.001745329252,
		    ],
                    angular_velocity: ImuVector3::from(gyroscope * (f64::consts::PI / 180.0)), 
		    angular_velocity_covariance: [
			0.001745329252, 0.0, 0.0,
			0.0, 0.001745329252, 0.0,
			0.0, 0.0, 0.001745329252,
		    ],
                    linear_acceleration: ImuVector3::from(accelerometer * G_TO_METERS_PER_SEC2),
		    linear_acceleration_covariance: [
			0.0784532, 0.0, 0.0,
			0.0, 0.0784532, 0.0,
			0.0, 0.0, 0.0784532,
		    ],
                })
            }
            sync_connection.sync_imu(&imu_messages).await;
        }
    }
}
