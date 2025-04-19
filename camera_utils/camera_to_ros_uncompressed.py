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
class Image(IdlStruct):
    header: Header
    height: uint32
    width: uint32
    encoding: str
    is_bigendian: uint8
    step: uint32
    data: bytes

if __name__ == '__main__':
    try:
        width = 1920
        height = 1080
        capture_interval = 0.5  # Capture every 0.5 seconds
        ros_topic = "qbot/camera/image"
        encoding = "YUV420"  # Or "bgr8" depending on PiCamera2 output
        frame_id = "camera_optical_frame"
        bytes_per_pixel = 1.5  # For RGB8 or BGR8

        # Zenoh setup
        with zenoh.open(zenoh.Config()) as session:
            publisher = session.declare_publisher(ros_topic)

            picam2 = Picamera2()
            config = picam2.create_still_configuration(main={"size": (width, height), "format": encoding})
            picam2.configure(config)
            picam2.start()

            print(f"Starting continuous capture at {width}x{height} in {encoding} format and publishing to Zenoh topic '{ros_topic}' (Ctrl+C to stop)...")

            while True:
                start_time = time.monotonic()
                image = picam2.capture_array() # Capture as a NumPy array
                image_bytes = image.tobytes()
                step = int(width * bytes_per_pixel)
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

                    image_msg = Image(
                        header=header,
                        height=height,
                        width=width,
                        encoding=encoding,
                        is_bigendian=0,
                        step=step,
                        data=image_bytes
                    )

                    # Serialize the message using pycdr2
                    serialized_data = image_msg.serialize()

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
