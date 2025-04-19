use std::time::{SystemTime, UNIX_EPOCH};
use libcamera::{CameraManager, Camera, StreamRole, StreamConfiguration, FrameBuffer, Request};
use libcamera::format::{Format, PixelFormat};
use libcamera::size::Size;
use zenoh::{prelude::*, config::Config, Session, Publisher, Bytes};
use std::error::Error;
use std::fmt;
use serde::{Serialize, Deserialize}; // For serializing ROS messages

// Define the custom error type
#[derive(Debug)]
enum CameraError {
    LibcameraError(libcamera::Error),
    ZenohError(zenoh::Error),
    Other(String),
}

impl fmt::Display for CameraError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            CameraError::LibcameraError(e) => write!(f, "Libcamera error: {}", e),
            CameraError::ZenohError(e) => write!(f, "Zenoh error: {}", e),
            CameraError::Other(s) => write!(f, "Camera error: {}", s),
        }
    }
}

impl Error for CameraError {}

impl From<libcamera::Error> for CameraError {
    fn from(e: libcamera::Error) -> Self {
        CameraError::LibcameraError(e)
    }
}

impl From<zenoh::Error> for CameraError {
    fn from(e: zenoh::Error) -> Self {
        CameraError::ZenohError(e)
    }
}

// Function to get a timestamp in microseconds
fn get_timestamp_us() -> u128 {
    let now = SystemTime::now();
    let duration = now.duration_since(UNIX_EPOCH).unwrap();
    duration.as_micros() as u128
}

// ROS message structs (defined according to your specifications)
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
    camera_manager.load()?;

    // Get the camera
    let camera_ids = camera_manager.camera_ids();
    if camera_ids.len() != 1 {
        return Err(CameraError::Other(format!(
            "Expected exactly one camera, found {}",
            camera_ids.len()
        )));
    }
    let mut camera = camera_manager.get(&camera_ids[0]).ok_or(CameraError::Other("Failed to get camera".to_string()))?;

    // 2. Configure the camera stream
    //   * Use MJPEG if libcamera supports it.  Check the libcamera documentation.
    //   * Set the resolution to 854x480.
    let stream_config = camera.generate_configuration(
        &[StreamRole::VideoRecording],
        Some(Size { width: 854, height: 480 }),
        Some(PixelFormat::MJPEG), // Try MJPEG.  If not supported, handle the error.
    )?;

    if stream_config.status() == libcamera::Status::Invalid {
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
    // 4. Start the camera
    camera.start()?;

    // 5. Zenoh setup
    let config = Config::default();
    let zenoh_session = zenoh::open(config)?;
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

            // Serialize the message
            let serialized_data = zenoh::serialization::serialize(&compressed_image_msg, zenoh::encoding::KeyExpr::from("application/cdr"))
                .map_err(|e| CameraError::ZenohError(e))?;

            // 9. Publish with Zenoh
            publisher.put(serialized_data)?;
            println!("Published compressed image at timestamp (us): {}", capture_timestamp_us);

            // 10. Requeue the buffer.
            camera.queue_frame_buffer(&captured_buffer)?;
        } else {
            eprintln!("No image planes available.");
        }
    }
    // 11. Cleanup
    camera.stop()?;
    camera.close()?;
    camera_manager.close()?;
    Ok(())
}
