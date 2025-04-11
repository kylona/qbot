use cdr_encoding::to_vec;
use zenoh::bytes::ZBytes;
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
    pub position: Point,
    pub orientation : Quaternion,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct Time {
    pub sec: i32,
    pub nsec: i32,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct Header {
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
    pub pose_publisher : Publisher<'a>,
}
impl SyncConnection<'_> {
    pub async fn default() -> Self {
         println!("Opening session...");
         let config = Config::default();
         let session = zenoh::open(config).await.unwrap();
         println!("Declaring Publisher on imu/pose'...");
         let pose_publisher = session.declare_publisher("imu/pose").await.unwrap();
         Self {
            session: session,
            pose_publisher : pose_publisher
         }
    }

    pub async fn sync_pose(&self, pose : Pose) {
        println!("GOT POSE: {:?}", pose);
        let stamped_pose = PoseStamped {
            header: Header {stamp: Time { sec: 0, nsec: 0 }, frame_id: String::from("map") },
            pose: pose,
        };
        //let payload_bytes = to_vec::<PoseStamped, LittleEndian>(&stamped_pose).unwrap();
        let mut payload_bytes = to_vec::<PoseStamped, LittleEndian>(&stamped_pose).unwrap();

	// Create the 4-byte CDR_LE header
        let header: [u8; 4] = [0x00, 0x01, 0x00, 0x00]; // CDR_LE identifier + zero options

        // 3. Prepend the header to the payload bytes
        let mut full_message_bytes = Vec::with_capacity(header.len() + payload_bytes.len());
        full_message_bytes.extend_from_slice(&header);
        full_message_bytes.append(&mut payload_bytes); // Note: append moves elements

        // 4. Convert to ZBytes for Zenoh
        let zbytes_payload: ZBytes = full_message_bytes.into();
        println!("SERIALIZED MESSAGE: {:?}", zbytes_payload);
        self.pose_publisher.put(zbytes_payload).await.unwrap();
    }
}

