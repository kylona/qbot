use std::time::{SystemTime, UNIX_EPOCH};
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

// fn main() -> Result<(), CameraError> {
//     // 1. Initialize libcamera
//     let camera_manager = CameraManager::new()?;
// 
//     // Get the camera
//     let cameras = camera_manager.cameras();
//     if cameras.len() != 1 {
//         return Err(CameraError::Other(format!(
//             "Expected exactly one camera, found {}",
//             cameras.len()
//         )));
//     }
//     let mut camera = cameras.get(0).unwrap();
// 
//     let stream_config = camera.generate_configuration(
//         &[StreamRole::VideoRecording],
//         Some(Size { width: 854, height: 480 }),
//         Some("jpeg"),
//     )?;
// 
//     if stream_config.status() == libcamera::camera::CameraConfigurationStatus::Invalid {
//         eprintln!("Invalid stream configuration!");
//         return Ok(()); // Or, you could return an error here if MJPEG is essential.
//     }
//     camera.configure(&stream_config)?;
//     let stream = stream_config.streams().next().ok_or(CameraError::Other("No stream".to_string()))?;
// 
//     // 3. Allocate frame buffers
//     let mut frame_buffers = Vec::new();
//     for _ in 0..stream.buffer_count() {
//         let buffer = camera.create_frame_buffer()?;
//         frame_buffers.push(buffer);
//     }
// 
//     for buffer in &frame_buffers {
//         camera.add_frame_buffer(buffer)?;
//     }
//     let active_camera = camera.activate()?;
// 
//     // 5. Zenoh setup
//     let config = Config::default();
//     let zenoh_session = zenoh::open(config).await.unwrap();
//     let publisher = zenoh_session.declare_publisher("qbot/camera/compressed_image")?; // Use the compressed image topic
// 
// 
//     loop {
//         // 6. Capture a frame
//         let mut request = camera.create_request()?;
//         if let Some(buffer) = frame_buffers.first() {
//             request.add_buffer(buffer)?;
//         }
//         request.queue()?;
// 
//         // Wait for the frame.
//         let captured_buffer = request.wait(std::time::Duration::from_millis(1000)).ok_or(CameraError::Other("Failed to capture frame within timeout".to_string()))?;
// 
//         // 7. Get timestamp
//         let capture_timestamp_us = get_timestamp_us();
//         let stamp_sec = (capture_timestamp_us / 1_000_000) as i32;
//         let stamp_nsec = ((capture_timestamp_us % 1_000_000) * 1_000) as i32;
// 
//         // 8. Process frame data and publish as CompressedImage
//         let planes = captured_buffer.planes();
//         if let Some(plane) = planes.first() {
//             let image_data = plane.data();
//             let image_size = plane.bytesused() as usize;
//             let image_bytes = Vec::from(image_data); // Convert to Vec
// 
//             // Construct the ROS CompressedImage message
//             let ros_time = Time { sec: stamp_sec, nsec: stamp_nsec };
//             let ros_header = Header {
//                 stamp: ros_time,
//                 frame_id: "camera_optical_frame".to_string(), // Use the correct frame_id
//             };
//             let compressed_image_msg = CompressedImage {
//                 header: ros_header,
//                 format: "jpeg".to_string(), // Use "jpeg" or "mjpeg" as appropriate. Libcamera should give you the right format.
//                 data: image_bytes,
//             };
// 
//             // Create the 4-byte CDR_LE header
//             let zenoh_header: [u8; 4] = [0x00, 0x01, 0x00, 0x00]; // CDR_LE identifier + zero options
// 
//             // Create the actual payload
//             let mut payload_bytes = to_vec::<CompressedImage, LittleEndian>(&compressed_image_msg).unwrap();
// 
//             // Prepend the header to the payload bytes
//             let mut full_message_bytes = Vec::with_capacity(zenoh_header.len() + payload_bytes.len());
//             full_message_bytes.extend_from_slice(&zenoh_header);
//             full_message_bytes.append(&mut payload_bytes); // Note: append moves elements
// 
//             // Convert to ZBytes for Zenoh
//             let zbytes_payload: ZBytes = full_message_bytes.into();
// 
//             // 9. Publish with Zenoh
//             publisher.put(zbytes_payload)?;
//             println!("Published compressed image at timestamp (us): {}", capture_timestamp_us);
// 
//         } else {
//             eprintln!("No image planes available.");
//         }
//     }
// }


use std::{fs::OpenOptions, io::Write, process::exit, time::Duration};

use libcamera::{
    geometry::{Size},
    camera::CameraConfigurationStatus,
    camera_manager::CameraManager,
    framebuffer::AsFrameBuffer,
    framebuffer_allocator::{FrameBuffer, FrameBufferAllocator},
    framebuffer_map::MemoryMappedFrameBuffer,
    pixel_format::PixelFormat,
    properties,
    request::ReuseFlag,
    stream::StreamRole,
};

// drm-fourcc does not have RGB888 type yet, construct it from raw fourcc identifier
const PIXEL_FORMAT_RGB888: PixelFormat = PixelFormat::new(875_710_290, 0);
const IMAGE_HEIGHT : usize = 854;
const IMAGE_WIDTH : usize = 480;

fn main() {
    let filename = match std::env::args().nth(1) {
        Some(f) => f,
        None => {
            println!("Error: missing file output parameter");
            println!("Usage: ./video_capture </path/to/output.mjpeg>");
            exit(1);
        }
    };

    let mgr = CameraManager::new().unwrap();

    let cameras = mgr.cameras();

    let cam = cameras.get(0).expect("No cameras found");

    println!(
        "Using camera: {}",
        *cam.properties().get::<properties::Model>().unwrap()
    );

    let mut cam = cam.acquire().expect("Unable to acquire camera");

    // This will generate default configuration for each specified role
    let mut cfgs = cam.generate_configuration(&[StreamRole::VideoRecording]).unwrap();
    println!("Camera Properties: {:#?}", cam.properties());


    cfgs.get_mut(0).unwrap().set_pixel_format(PIXEL_FORMAT_RGB888);
    cfgs.get_mut(0).unwrap().set_size(Size {
      width: IMAGE_WIDTH as u32,
      height: IMAGE_HEIGHT as u32,
    });

    println!("Generated config: {:#?}", cfgs);

    match cfgs.validate() {
        CameraConfigurationStatus::Valid => println!("Camera configuration valid!"),
        CameraConfigurationStatus::Adjusted => println!("Camera configuration was adjusted: {:#?}", cfgs),
        CameraConfigurationStatus::Invalid => panic!("Error validating camera configuration"),
    }

    // Ensure that pixel format was unchanged
    assert_eq!(
        cfgs.get(0).unwrap().get_pixel_format(),
        PIXEL_FORMAT_RGB888,
        "RGB888 is not supported by the camera"
    );

    cam.configure(&mut cfgs).expect("Unable to configure camera");

    let mut alloc = FrameBufferAllocator::new(&cam);

    // Allocate frame buffers for the stream
    let cfg = cfgs.get(0).unwrap();
    let stream = cfg.stream().unwrap();
    let buffers = alloc.alloc(&stream).unwrap();
    println!("Allocated {} buffers", buffers.len());

    // Convert FrameBuffer to MemoryMappedFrameBuffer, which allows reading &[u8]
    let buffers = buffers
        .into_iter()
        .map(|buf| MemoryMappedFrameBuffer::new(buf).unwrap())
        .collect::<Vec<_>>();

    // Create capture requests and attach buffers
    let reqs = buffers
        .into_iter()
        .enumerate()
        .map(|(i, buf)| {
            let mut req = cam.create_request(Some(i as u64)).unwrap();
            req.add_buffer(&stream, buf).unwrap();
            req
        })
        .collect::<Vec<_>>();

    // Completed capture requests are returned as a callback
    let (tx, rx) = std::sync::mpsc::channel();
    cam.on_request_completed(move |req| {
        tx.send(req).unwrap();
    });

    // TODO: Set `Control::FrameDuration()` here. Blocked on https://github.com/lit-robotics/libcamera-rs/issues/2
    cam.start(None).unwrap();

    // Enqueue all requests to the camera
    for req in reqs {
        println!("Request queued for execution: {req:#?}");
        cam.queue_request(req).unwrap();
    }

    let mut file = OpenOptions::new()
        .append(true)
        .create(true)
        .open(&filename)
        .expect("Unable to create output file");
    let mut count = 0;
    let mut jpeg = Vec::new();
    while count < 10 {

        println!("Waiting for camera request execution");
        let mut req = rx.recv_timeout(Duration::from_secs(2)).expect("Camera request failed");

        println!("Camera request {:?} completed!", req);
        println!("Metadata: {:#?}", req.metadata());

        // Get framebuffer for our stream
        let framebuffer: &MemoryMappedFrameBuffer<FrameBuffer> = req.buffer(&stream).unwrap();
        println!("FrameBuffer metadata: {:#?}", framebuffer.metadata());

        // RGB888 format has only one data plane containing encoded jpeg data with all the headers
        let planes = framebuffer.data();
        let frame_data = planes.get(0).unwrap();
        // Actual encoded data will be smalled than framebuffer size, its length can be obtained from metadata.
        let bytes_used = framebuffer.metadata().unwrap().planes().get(0).unwrap().bytes_used as usize;

        let mut encoder = image::codecs::jpeg::JpegEncoder::new(&mut jpeg);

        encoder.encode(&frame_data, IMAGE_WIDTH as u32, IMAGE_HEIGHT as u32, image::ExtendedColorType::Rgb8).unwrap();
				println!("Written {} bytes to buffer", bytes_used);

        file.write_all(&jpeg).unwrap();
				println!("Written {} bytes to {}", jpeg.len(), &filename);

        // Recycle the request back to the camera for execution
        req.reuse(ReuseFlag::REUSE_BUFFERS);
        cam.queue_request(req).unwrap();

        count += 1;
    }

    // Everything is cleaned up automatically by Drop implementations
}

