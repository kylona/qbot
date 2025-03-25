extern crate mpu9250_fifo;
use mpu9250_fifo::mpu9250::{MPU9250};
use mpu9250_fifo::mpu9250::{AccelerometerMeasurement, GyroscopeMeasurement, MagnetometerMeasurement};
use ahrs::{Ahrs, Madgwick};
use nalgebra::Vector3;
use simpsons::SimpsonsIntegral;
use std::f64;
use std::io::Write;
use std::io::stdout;

const G_TO_METERS_PER_SEC2 : f32 = 9.80665;

pub mod simpsons {
    use crate::G_TO_METERS_PER_SEC2;

    use super::AccelerometerMeasurement;

    pub struct SimpsonsIntegral {
        pub current_sum : f32,
        delta_t : f32,
        floating_measurement : Option<AccelerometerMeasurement>,
    }
    impl SimpsonsIntegral {
        pub fn default() -> Self {
            Self::new(1.0/500.0, 0.0)
        }
        pub fn new(delta_t : f32, current_sum : f32) -> Self {
            Self {
                current_sum: current_sum,
                delta_t: delta_t,
                floating_measurement : None
            }
        }
        pub fn update(&mut self, accel_meas : &[AccelerometerMeasurement]) -> f32 {
            let first : f32;
            let last : f32;
            let divisor = 1.0/3.0*self.delta_t * G_TO_METERS_PER_SEC2;
            let mut odd_sum : f32 = 0.0;
            let mut even_sum : f32 = 0.0;
            if accel_meas.len() < 3 {
                let mut sum = 0.0;
                for i in (0..accel_meas.len()).step_by(2) {
                    sum += accel_meas[i].x;
                }
                self.current_sum += divisor * sum;
                return self.current_sum;
            }
            if accel_meas.len() % 2 == 0 {
                // If we have an even number of points
                first = accel_meas[0].x;
                last = accel_meas[accel_meas.len() - 1].x;
                for i in (1..accel_meas.len()-1).step_by(2) {
                    odd_sum += accel_meas[i].x;
                    even_sum += accel_meas[i+1].x;
                }
            }
            else {
                match self.floating_measurement {
                    Some(measure) => {
                        first = measure.x;
                        last = accel_meas[accel_meas.len() - 1].x;
                        for i in (1..accel_meas.len()-1).step_by(2) {
                            odd_sum += accel_meas[i].x;
                            even_sum += accel_meas[i+1].x;
                        }
                        self.floating_measurement = None
                    }
                    None => {
                        first = accel_meas[0].x;
                        last = accel_meas[accel_meas.len() - 2].x;
                        for i in (1..(accel_meas.len()-2)).step_by(2) {
                            odd_sum += accel_meas[i].x;
                            even_sum += accel_meas[i+1].x;
                        }
                        self.floating_measurement = Some(accel_meas[accel_meas.len() - 1]);

                    }
                }
            }
            self.current_sum += divisor * (first + 4.0*odd_sum + 2.0*even_sum + last);
            self.current_sum
    }


}

}


fn main() {
    let mut accel_meas = [AccelerometerMeasurement { x: 0.0, y : 0.0, z: 0.0}; 100];
    let mut gyro_meas = [GyroscopeMeasurement { x: 0.0, y : 0.0, z: 0.0}; 100];
    let mut mag_meas = [MagnetometerMeasurement { x: 0.0, y : 0.0, z: 0.0}; 100];

    // Initialize filter with default values
    let mut ahrs = Madgwick::new(1.0/500.0, 0.1);
    let mut simpsons = SimpsonsIntegral::new(1.0/500.0, 0.0);
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
        simpsons.update(&accel_meas);
        std::thread::sleep(std::time::Duration::from_millis(10));
        let (roll, pitch, yaw) = quat.euler_angles();
        // Do something with the updated state quaternion
        print!("x_vel={:0.5}, pitch={:0.5}, roll={:0.5}, yaw={:0.5}\r", simpsons.current_sum, pitch * 180.0 /f64::consts::PI, roll * 180.0 /f64::consts::PI, yaw * 180.0 /f64::consts::PI);
        stdout().flush().expect("Flush std out failed");
    }
}
