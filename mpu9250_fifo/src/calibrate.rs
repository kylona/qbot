use crate::mpu9250;
use anyhow::Result;

const GYRO_NUM_SAMPLES: i64 = 5000;
const ACCEL_NUM_SAMPLES: i64 = 2000;
const MAG_NUM_SAMPLES: i64 = 2000;
const ONE_G_FOR_2_G_FULL_SCALE : i64 = 16385;

pub fn wait(num_seconds : u8) -> () {
    for i in 0..num_seconds {
        println!("Starting in {} seconds", num_seconds - i); 
        std::thread::sleep(std::time::Duration::from_secs(1));
    }
}

pub fn calibrate_gyroscope(mpu9250 : &mut mpu9250::MPU9250) -> Result<mpu9250::GyroscopeMeasurement> {
    println!("Calibrating Gyroscope: Keep the MPU9250 very still");
    wait(3);
    println!("Calibrating . . .");

    let mut gyro_sum_x : i64 = 0;
    let mut gyro_sum_y : i64 = 0;
    let mut gyro_sum_z : i64 = 0;
    for _ in 0..GYRO_NUM_SAMPLES {
        let gyro_data = mpu9250.get_rotation()?;
        gyro_sum_x += gyro_data.x as i64;
        gyro_sum_y += gyro_data.y as i64;
        gyro_sum_z += gyro_data.z as i64;
    }
    let mut gyro_offset_x = f32::from(gyro_sum_x) / f32::from(GYRO_NUM_SAMPLES);
    let mut gyro_offset_y = f32::from(gyro_sum_x) / f32::from(GYRO_NUM_SAMPLES);
    let mut gyro_offset_z = f32::from(gyro_sum_x) / f32::from(GYRO_NUM_SAMPLES);
    println!("Calibration Complete.");
    let gyro_offset = mpu9250::GyroscopeMeasurement {
        x: gyro_offset_x,
        y: gyro_offset_y,
        z: gyro_offset_z,
    };
    println!("GYRO OFFSET IS: {:?}", gyro_offset);
    Ok(gyro_offset)
}

pub fn calibrate_accelerometer(mpu9250 : &mut mpu9250::MPU9250) -> Result<mpu9250::AccelerometerMeasurement> {
    println!("Calibrating Accelerometer: Keep the MPU9250 still with z axis pointed up");
    wait(3);
    println!("Calibrating . . .");

    let mut accel_offset_x : i64 = 0;
    let mut accel_offset_y : i64 = 0;
    let mut accel_offset_z : i64 = 0;
    for _ in 0..ACCEL_NUM_SAMPLES {
        let accel_data = mpu9250.get_acceleration()?;
        accel_offset_x += accel_data.x as i64;
        accel_offset_y += accel_data.y as i64;
        accel_offset_z += accel_data.z as i64;
    }
    accel_offset_x /= ACCEL_NUM_SAMPLES;
    accel_offset_y /= ACCEL_NUM_SAMPLES;
    accel_offset_z /= ACCEL_NUM_SAMPLES;
    // Handle either z axis up or z axis down
    if accel_offset_z > 0 {
        accel_offset_z -= ONE_G_FOR_2_G_FULL_SCALE;
    }
    else {
        accel_offset_z += ONE_G_FOR_2_G_FULL_SCALE;
    }
    println!("Calibration Complete.");
    let accel_offset = mpu9250::AccelerometerMeasurement {
        x: accel_offset_x.try_into().expect("X accel offset greater than full scale"),
        y: accel_offset_y.try_into().expect("Y accel offset greater than full scale"),
        z: accel_offset_z.try_into().expect("Z accel offset greater than full scale"),
    };
    println!("ACCEL OFFSET IS: {:?}", accel_offset);
    Ok(accel_offset)
}

pub fn calibrate_magnetometer(mpu9250 : &mut mpu9250::MPU9250) -> Result<(mpu9250::MagnetometerMeasurement, mpu9250::MagnetometerMeasurement)> {
    println!("Calibrating Magnetometer: Rotate the MPU9250 so each axis faces north at least once.");
    wait(3);
    println!("Calibrating . . .");

    let mag_data = mpu9250.get_magnetometer_data()?;
    // 80% of the norm of the 3d vector
    let target_scale : i16 = (f64::sqrt(((mag_data.x as i64).pow(2) + (mag_data.y as i64).pow(2) + (mag_data.z as i64).pow(2)) as f64)*0.8).round() as i16;

    let mut mag_min_x : i16 = 0;
    let mut mag_max_x : i16 = 0;
    let mut mag_min_y : i16 = 0;
    let mut mag_max_y : i16 = 0;
    let mut mag_min_z : i16 = 0;
    let mut mag_max_z : i16 = 0;
    let mut x_done = false;
    let mut y_done = false;
    let mut z_done = false;
    for _ in 0..MAG_NUM_SAMPLES {
        let mag_data = mpu9250.get_magnetometer_data()?;
        mag_min_x = std::cmp::min(mag_data.x, mag_min_x);
        mag_max_x = std::cmp::max(mag_data.x, mag_max_x);
        mag_min_y = std::cmp::min(mag_data.y, mag_min_y);
        mag_max_y = std::cmp::max(mag_data.y, mag_max_y);
        mag_min_z = std::cmp::min(mag_data.z, mag_min_z);
        mag_max_z = std::cmp::max(mag_data.z, mag_max_z);
        if !x_done && (mag_min_x < -target_scale) && (mag_max_x > target_scale) {
            x_done = true;
            println!("X Axis Calibrated")
        }
        if !y_done && (mag_min_y < -target_scale) && (mag_max_y > target_scale) {
            y_done = true;
            println!("Y Axis Calibrated")
        }
        if !z_done && (mag_min_z < -target_scale) && (mag_max_z > target_scale) {
            z_done = true;
            println!("Z Axis Calibrated")
        }
        if x_done && y_done && z_done {
            break
        }
    }
    let mag_avg = mpu9250::MagnetometerMeasurement {
        x: f32::from(mag_max_x - mag_min_x) / 2.0,
        y: f32::from(mag_max_y - mag_min_y) / 2.0,
        z: f32::from(mag_max_z - mag_min_z) / 2.0,
    };
    let avg_radius = (mag_avg.x + mag_avg.y + mag_avg.z) / 3.0;

    let mag_offset = mpu9250::MagnetometerMeasurement {
        x: (mag_max_x + mag_min_x) / 2,
        y: (mag_max_y + mag_min_y) / 2,
        z: (mag_max_z + mag_min_z) / 2,
    };
    println!("mag_avg: {:?}", mag_avg);
    println!("mag_offset: {:?}", mag_offset);
    let mag_scale = mpu9250::MagnetometerMeasurement {
        x: avg_radius / mag_avg.x,
        y: avg_radius / mag_avg.y,
        z: avg_radius / mag_avg.z,
    };
    Ok((mag_offset, mag_scale))
}
