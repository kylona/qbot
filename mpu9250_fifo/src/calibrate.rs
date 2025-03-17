use crate::mpu9250;
use anyhow::Result;

const GYRO_NUM_SAMPLES: i64 = 5000;
const ACCEL_NUM_SAMPLES: i64 = 2000;
const MAG_NUM_SAMPLES: i64 = 2000;

pub fn wait(num_seconds : u8) -> () {
    for i in 0..num_seconds {
        println!("Starting in {} seconds", num_seconds - i); 
        std::thread::sleep(std::time::Duration::from_secs(1));
    }
}

pub fn calibrate_gyro(mpu9250 : &mut mpu9250::MPU9250) -> Result<mpu9250::GyroscopeData> {
    println!("Calibrating Gyroscope: Keep the MPU9250 very still");
    wait(3);
    println!("Calibrating . . .");

    let mut gyro_offset_x : i64 = 0;
    let mut gyro_offset_y : i64 = 0;
    let mut gyro_offset_z : i64 = 0;
    for _ in 0..GYRO_NUM_SAMPLES {
        let gyro_data = mpu9250.get_rotation()?;
        gyro_offset_x += gyro_data.x as i64;
        gyro_offset_y += gyro_data.y as i64;
        gyro_offset_z += gyro_data.z as i64;
    }
    gyro_offset_x /= GYRO_NUM_SAMPLES;
    gyro_offset_y /= GYRO_NUM_SAMPLES;
    gyro_offset_z /= GYRO_NUM_SAMPLES;
    println!("Calibration Complete.");
    let gyro_offset = mpu9250::GyroscopeData {
        x: gyro_offset_x.try_into().expect("X gyro offset greater than full scale"),
        y: gyro_offset_y.try_into().expect("Y gyro offset greater than full scale"),
        z: gyro_offset_z.try_into().expect("Z gyro offset greater than full scale"),
    };
    println!("GYRO OFFSET IS: {:?}", gyro_offset);
    Ok(gyro_offset)
}
