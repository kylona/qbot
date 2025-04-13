use cdr_encoding::to_vec;
use zenoh::bytes::ZBytes;
use zenoh::{Session, Config};
use zenoh::pubsub::Publisher;
use serde::{Serialize, Deserialize};
use byteorder::LittleEndian;
use nalgebra::Vector3;

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct Quaternion {
    pub x : f64,
    pub y : f64,
    pub z : f64,
    pub w : f64,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ImuVector3 {
    pub x : f64,
    pub y : f64,
    pub z : f64,
}
impl ImuVector3 {
    fn default() -> Self {
        return Self {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }
    }
}
impl From<Vector3<f64>> for ImuVector3 {
    fn from(item: Vector3<f64>) -> Self {
        ImuVector3 {
            x: item[0],
            y: item[1],
            z: item[2],
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct Time {
    pub sec: i32,
    pub nsec: i32,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct Header {
    pub stamp : Time,
    pub frame_id : String,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct Imu {
    pub header : Header,
    pub orientation : Quaternion,
    pub orientation_covariance : [f64; 9],
    pub angular_velocity : ImuVector3,
    pub angular_velocity_covariance : [f64; 9],
    pub linear_acceleration : ImuVector3,
    pub linear_acceleration_covariance : [f64; 9],
}
impl Imu {
    pub fn default() -> Self {
	return Self {
            header: Header {
               stamp: Time {
                    sec: 0,
                    nsec: 0,
               },
               frame_id: String::from("qbot")
            },
            orientation: Quaternion {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 0.0,
            },
            orientation_covariance: [
                0.1, 0.0, 0.0,
                0.0, 0.1, 0.0,
                0.0, 0.0, 0.1,
            ],
            angular_velocity: ImuVector3::default(),
            angular_velocity_covariance: [
                0.1, 0.0, 0.0,
                0.0, 0.1, 0.0,
                0.0, 0.0, 0.1,
            ],
            linear_acceleration: ImuVector3::default(),
            linear_acceleration_covariance: [
                0.0784532, 0.0, 0.0,
                0.0, 0.0784532, 0.0,
                0.0, 0.0, 0.0784532,
            ],
        }
    }
}
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ImuPack {
    pub size : u32,
    pub data : Vec<Imu>,
}
impl From<&Vec<Imu>> for ImuPack {
    fn from(data: &Vec<Imu>) -> Self {
        Self {
            size: data.len() as u32,
            data: data.to_vec(),
        }
    }
}


pub struct SyncConnection<'a> {
    pub session : Session,
    pub imu_publisher : Publisher<'a>,
}
impl SyncConnection<'_> {
    pub async fn default() -> Self {
         println!("Opening session...");
         let config = Config::default();
         let session = zenoh::open(config).await.unwrap();
         println!("Declaring Publisher on qbot/imu'...");
         let imu_publisher = session.declare_publisher("qbot/imu").await.unwrap();
         Self {
            session: session,
            imu_publisher : imu_publisher
         }
    }

    pub async fn sync_imu(&self, imu_messages : &Vec<Imu>) {

        for message in imu_messages {
            // Create the 4-byte CDR_LE header
            let header: [u8; 4] = [0x00, 0x01, 0x00, 0x00]; // CDR_LE identifier + zero options

            // Create the actual payload
            let mut payload_bytes = to_vec::<Imu, LittleEndian>(message).unwrap();
            //
            // Prepend the header to the payload bytes
            let mut full_message_bytes = Vec::with_capacity(header.len() + payload_bytes.len());
            full_message_bytes.extend_from_slice(&header);
            full_message_bytes.append(&mut payload_bytes); // Note: append moves elements

            // Convert to ZBytes for Zenoh
            let zbytes_payload: ZBytes = full_message_bytes.into();
            self.imu_publisher.put(zbytes_payload).await.unwrap();
        }
    }

}

