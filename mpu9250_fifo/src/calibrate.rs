use crate::mpu9250;
use anyhow::Result;

const GYRO_NUM_SAMPLES: i16 = 5000;
const ACCEL_NUM_SAMPLES: i16 = 2000;
const MAG_NUM_SAMPLES: i16 = 2000;

pub fn calibrate_gyro(mpu9250 : &mut mpu9250::MPU9250) -> Result<()> {
    let mut gyro_offset = mpu9250::GyroscopeData {
        x: 0,
        y: 0,
        z: 0,
    };
    for _ in 0..GYRO_NUM_SAMPLES {
        let gyro_data = mpu9250.get_rotation()?;
        gyro_offset.x += gyro_data.x;
        gyro_offset.y += gyro_data.y;
        gyro_offset.z += gyro_data.z;
    }
    gyro_offset.x /= GYRO_NUM_SAMPLES;
    gyro_offset.y /= GYRO_NUM_SAMPLES;
    gyro_offset.z /= GYRO_NUM_SAMPLES;
    println!("GYRO OFFSET IS: {:?}", gyro_offset);
    Ok(())
}