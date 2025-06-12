use std::time::{SystemTime, UNIX_EPOCH};
use libcamera::camera_manager::{CameraManager};
use libcamera::stream::{StreamRole};
use libcamera::pixel_format::{PixelFormat};
use libcamera::geometry::Size;
use zenoh::bytes::ZBytes;
use zenoh::{config::Config};
use std::error::Error;
use std::fmt;
use serde::{Serialize, Deserialize}; // For serializing ROS messages
use byteorder::LittleEndian;
use cdr_encoding::to_vec;

// Define the custom error type
#[derive(Debug)]
enum CameraError {
    ZenohError(zenoh::Error),
    IoError(std::io::Error),
    Other(String),
}

impl fmt::Display for CameraError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            CameraError::ZenohError(e) => write!(f, "Zenoh error: {}", e),
            CameraError::IoError(e) => write!(f, "IO error: {}", e),
            CameraError::Other(s) => write!(f, "Camera error: {}", s),
        }
    }
}

impl Error for CameraError {}

impl From<zenoh::Error> for CameraError {
    fn from(e: zenoh::Error) -> Self {
        CameraError::ZenohError(e)
    }
}

impl From<std::io::Error> for CameraError {
    fn from(e: std::io::Error) -> Self {
        CameraError::IoError(e)
    }
}

// Function to get a timestamp in microseconds
fn get_timestamp_us() -> u128 {
    let now = SystemTime::now();
    let duration = now.duration_since(UNIX_EPOCH).unwrap();
    duration.as_micros() as u128
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct Time {
    pub sec: i32,
    pub nsec: i32,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct Header {
    stamp: Time,
    frame_id: String,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct CompressedImage {
    pub header: Header,
    pub format: String,
    pub data: Vec<u8>, // Use Vec<u8> for byte data
}

fn main() -> Result<(), CameraError> {
    // 1. Initialize libcamera
    let camera_manager = CameraManager::new()?;

    // Get the camera
    let cameras = camera_manager.cameras();
    if cameras.len() != 1 {
        return Err(CameraError::Other(format!(
            "Expected exactly one camera, found {}",
            cameras.len()
        )));
    }
    let mut camera = cameras.get(0).unwrap();

    let stream_config = camera.generate_configuration(
        &[StreamRole::VideoRecording],
        Some(Size { width: 854, height: 480 }),
        Some("jpeg"),
    )?;

    if stream_config.status() == libcamera::camera::CameraConfigurationStatus::Invalid {
        eprintln!("Invalid stream configuration!");
        return Ok(()); // Or, you could return an error here if MJPEG is essential.
    }
    camera.configure(&stream_config)?;
    let stream = stream_config.streams().next().ok_or(CameraError::Other("No stream".to_string()))?;

    // 3. Allocate frame buffers
    let mut frame_buffers = Vec::new();
    for _ in 0..stream.buffer_count() {
        let buffer = camera.create_frame_buffer()?;
        frame_buffers.push(buffer);
    }

    for buffer in &frame_buffers {
        camera.add_frame_buffer(buffer)?;
    }
    let active_camera = camera.activate()?;

    // 5. Zenoh setup
    let config = Config::default();
    let zenoh_session = zenoh::open(config).await.unwrap();
    let publisher = zenoh_session.declare_publisher("qbot/camera/compressed_image")?; // Use the compressed image topic


    loop {
        // 6. Capture a frame
        let mut request = camera.create_request()?;
        if let Some(buffer) = frame_buffers.first() {
            request.add_buffer(buffer)?;
        }
        request.queue()?;

        // Wait for the frame.
        let captured_buffer = request.wait(std::time::Duration::from_millis(1000)).ok_or(CameraError::Other("Failed to capture frame within timeout".to_string()))?;

        // 7. Get timestamp
        let capture_timestamp_us = get_timestamp_us();
        let stamp_sec = (capture_timestamp_us / 1_000_000) as i32;
        let stamp_nsec = ((capture_timestamp_us % 1_000_000) * 1_000) as i32;

        // 8. Process frame data and publish as CompressedImage
        let planes = captured_buffer.planes();
        if let Some(plane) = planes.first() {
            let image_data = plane.data();
            let image_size = plane.bytesused() as usize;
            let image_bytes = Vec::from(image_data); // Convert to Vec

            // Construct the ROS CompressedImage message
            let ros_time = Time { sec: stamp_sec, nsec: stamp_nsec };
            let ros_header = Header {
                stamp: ros_time,
                frame_id: "camera_optical_frame".to_string(), // Use the correct frame_id
            };
            let compressed_image_msg = CompressedImage {
                header: ros_header,
                format: "jpeg".to_string(), // Use "jpeg" or "mjpeg" as appropriate. Libcamera should give you the right format.
                data: image_bytes,
            };

            // Create the 4-byte CDR_LE header
            let zenoh_header: [u8; 4] = [0x00, 0x01, 0x00, 0x00]; // CDR_LE identifier + zero options

            // Create the actual payload
            let mut payload_bytes = to_vec::<CompressedImage, LittleEndian>(&compressed_image_msg).unwrap();

            // Prepend the header to the payload bytes
            let mut full_message_bytes = Vec::with_capacity(zenoh_header.len() + payload_bytes.len());
            full_message_bytes.extend_from_slice(&zenoh_header);
            full_message_bytes.append(&mut payload_bytes); // Note: append moves elements

            // Convert to ZBytes for Zenoh
            let zbytes_payload: ZBytes = full_message_bytes.into();

            // 9. Publish with Zenoh
            publisher.put(zbytes_payload)?;
            println!("Published compressed image at timestamp (us): {}", capture_timestamp_us);

        } else {
            eprintln!("No image planes available.");
        }
    }
}
