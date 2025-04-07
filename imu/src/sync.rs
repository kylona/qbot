use cdr_encoding::to_vec;
use zenoh::{Session, Config};
use zenoh::pubsub::Publisher;
use serde::{Serialize, Deserialize};
use byteorder::LittleEndian;

#[derive(Serialize, Deserialize, Debug)]
pub struct Quaternion {
    pub x : f64,
    pub y : f64,
    pub z : f64,
    pub w : f64,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct Point {
    pub x : f64,
    pub y : f64,
    pub z : f64,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct Pose {
    pub orientation : Quaternion,
    pub position: Point,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct Time {
    pub sec: i32,
    pub nsec: i32,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct Header {
    pub seq : i32,
    pub stamp : Time,
    pub frame_id : String,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct PoseStamped {
    header : Header,
    pose : Pose,
}

pub struct SyncConnection<'a> {
    pub session : Session,
    pub orientation_publisher : Publisher<'a>,
}
impl SyncConnection<'_> {
    pub async fn default() -> Self {
         println!("Opening session...");
         let config = Config::default();
         let session = zenoh::open(config).await.unwrap();
         println!("Declaring Publisher on imu/orientation'...");
         let orientation_publisher = session.declare_publisher("imu/orientation").await.unwrap();
         Self {
            session: session,
            orientation_publisher : orientation_publisher
         }
    }

    pub async fn sync_pose(&self, pose : Pose) {
        println!("GOT POSE: {:?}", pose);
        let stamped_pose = PoseStamped {
            header: Header { seq: 0, stamp: Time { sec: 0, nsec: 0 }, frame_id: String::from("qbot") },
            pose: pose,
        };
        let serialized = to_vec::<PoseStamped, LittleEndian>(&stamped_pose).unwrap();
        println!("SERIALIZED MESSAGE: {:?}", serialized);
        self.orientation_publisher.put(serialized).await.unwrap();
    }
}

