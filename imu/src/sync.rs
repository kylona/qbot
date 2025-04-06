 use cdr_encoding::to_vec;
use zenoh::{}
use serde::{Serialize, Deserialize};
use byteorder::LittleEndian;

pub struct Quaternion {
    x : f64,
    y : f64,
    z : f64,
    w : f64,
}

pub struct Point {
    x : f64,
    y : f64,
    z : f64,
}

pub struct Pose {
    orientation : Quaternion,
    position: Point,
}

pub struct Time {
    sec: i32,
    nsec: i32,
}

pub struct Header {
    seq : i32,
    stamp : Time,
    frame_id : str,
}

pub struct SyncConnection {
    session : zenoh::Session
    orientation_publisher : zenoh::Publisher
}
impl SyncConnection {
    pub fn default() -> Self {
         println!("Opening session...");
         config = zenoh::Config.from_json5("");
         let session = zenoh::open(config).await.unwrap();
         println!("Declaring Publisher on imu/orientation'...");
         let orientation_publisher = session.declare_publisher("imu/orientation").await.unwrap();
         Self {
            session: session
            orientation_publisher : orientation_publisher
         }
    }

    pub fn sync_pose(&self, pose : Pose) {
        println!("GOT POSE: {:?}", pose)
        let serialized = to_vec::<ShapeType, LittleEndian>(&message).unwrap();
        println!("SERIALIZED MESSAGE: {:?}", serialized)
    }
}

