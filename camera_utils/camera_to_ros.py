from picamera2 import Picamera2
import io
import time
from dataclasses import dataclass
from pycdr2 import IdlStruct
from pycdr2.types import uint32, uint8, int32
import zenoh

# Define the ROS 2 message structures using pycdr2
@dataclass
class Time(IdlStruct):
    sec: int32
    nsec: uint32

@dataclass
class Header(IdlStruct):
    stamp: Time
    frame_id: str

@dataclass
class CompressedImage(IdlStruct):
    header: Header
    format: str
    data: bytes

if __name__ == '__main__':
    try:
        width = 854 
        height = 480
        capture_interval = 1/30
        ros_topic = "qbot/camera/compressed_image"
        image_format = "jpeg"  
        frame_id = "camera_optical_frame"

        # Zenoh setup
        with zenoh.open(zenoh.Config()) as session:
            publisher = session.declare_publisher(ros_topic)

            picam2 = Picamera2()
            config = picam2.create_video_configuration(main={"size": (width, height)})
            picam2.configure(config)
            picam2.start()

            print(f"Starting continuous capture at {width}x{height} in {image_format} format and publishing to Zenoh topic '{ros_topic}' (Ctrl+C to stop)...")

            while True:
                start_time = time.monotonic()
                stream = io.BytesIO()
                picam2.capture_file(stream, format=image_format)
                image_bytes = stream.getvalue()
                end_time = time.monotonic()
                capture_duration = end_time - start_time

                if image_bytes:
                    # Create the ROS 2 message components
                    now = time.time()
                    stamp_sec = int(now)
                    stamp_nanosec = int((now - stamp_sec) * 1e9)

                    header = Header(
                        stamp=Time(sec=stamp_sec, nsec=stamp_nanosec),
                        frame_id=frame_id
                    )

                    compressed_image_msg = CompressedImage(
                        header=header,
                        format=image_format,
                        data=image_bytes
                    )

                    # Serialize the message using pycdr2
                    serialized_data = compressed_image_msg.serialize()

                    # Publish the serialized data over Zenoh
                    publisher.put(serialized_data)
                    print(f"Published {len(serialized_data)} bytes to '{ros_topic}' in {capture_duration:.2f} seconds.")
                else:
                    print("Failed to capture image.")

                sleep_time = capture_interval - capture_duration
                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    print(f"Warning: Capture took longer than the interval ({capture_duration:.2f} > {capture_interval:.2f}).")

    except KeyboardInterrupt:
        print("\nContinuous capture stopped by user.")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        if 'picam2' in locals() and picam2.is_open:
            picam2.close()
