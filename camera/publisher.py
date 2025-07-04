import io
import time
import zenoh
from picamera2 import Picamera2
from picamera2.encoders import MJPEGEncoder
from picamera2.outputs import FileOutput
from dataclasses import dataclass
from pycdr2 import IdlStruct
from pycdr2.types import uint32, int32

# =================================================================
# 1. ROS 2 Message Structure Definitions (using pycdr2)
# These data classes define the structure of the message that will
# be sent over Zenoh. This ensures compatibility with ROS 2 listeners.
# =================================================================

@dataclass
class Time(IdlStruct):
    """Represents time in seconds and nanoseconds."""
    sec: int32
    nsec: uint32

@dataclass
class Header(IdlStruct):
    """Represents standard ROS 2 message header."""
    stamp: Time
    frame_id: str

@dataclass
class CompressedImage(IdlStruct):
    """Represents a ROS 2 CompressedImage message."""
    header: Header
    format: str
    data: bytes

# =================================================================
# 2. Custom Picamera2 Output for Zenoh
# This class acts as the target for the camera's encoder. Instead of
# writing to a file, its `write` method publishes the frame data
# directly to a Zenoh topic.
# =================================================================

class ZenohOutput(io.BufferedIOBase):
    """
    A custom Picamera2 output class that publishes frames to a Zenoh topic.
    It formats each frame into a ROS 2 CompressedImage message before sending.
    """
    def __init__(self, publisher: zenoh.Publisher, frame_id: str, image_format: str):
        """
        Initializes the Zenoh output.

        Args:
            publisher: The Zenoh publisher instance.
            frame_id: The camera frame ID for the ROS message header.
            image_format: The format of the image data (e.g., 'jpeg').
        """
        self.publisher = publisher
        self.frame_id = frame_id
        self.image_format = image_format
        self.frame_count = 0
        super().__init__()

    def write(self, b: bytes) -> int:
        """
        This method is called by the Picamera2 encoder for each new frame.

        Args:
            b: The raw byte buffer of the compressed frame (e.g., a JPEG image).
        
        Returns:
            The number of bytes written (published).
        """
        # Create the ROS 2 message timestamp
        now = time.time()
        stamp_sec = int(now)
        stamp_nanosec = int((now - stamp_sec) * 1e9)

        # Assemble the message header
        header = Header(
            stamp=Time(sec=stamp_sec, nsec=stamp_nanosec),
            frame_id=self.frame_id
        )

        # Assemble the full CompressedImage message
        compressed_image_msg = CompressedImage(
            header=header,
            format=self.image_format,
            data=b  # The incoming buffer is the image data
        )

        # Serialize the message to a byte array using pycdr2
        serialized_data = compressed_image_msg.serialize()

        # Publish the serialized data over Zenoh
        self.publisher.put(serialized_data)
        
        self.frame_count += 1
        print(f"Published frame {self.frame_count} ({len(serialized_data)} bytes)", end='\r')
        
        # The write method must return the number of bytes "written"
        return len(b)

# =================================================================
# 3. Main Application Logic
# This section sets up the camera, configures Zenoh, and starts
# the recording process, tying everything together.
# =================================================================

if __name__ == '__main__':
    # --- Configuration ---
    WIDTH = 854
    HEIGHT = 480
    FRAMERATE = 50
    BITRATE = 8000000  # 8 Mbps, adjust as needed for quality vs. bandwidth
    ZENOH_TOPIC = "qbot/camera/compressed_image"
    IMAGE_FORMAT = "jpeg"
    FRAME_ID = "camera_optical_frame"

    # Initialize Picamera2
    picam2 = Picamera2()
    
    # --- Zenoh Setup ---
    print("Opening Zenoh session...")
    conf = zenoh.Config()
    session = zenoh.open(conf)
    publisher = session.declare_publisher(ZENOH_TOPIC)
    
    try:
        # --- Camera Configuration ---
        print(f"Configuring camera for {WIDTH}x{HEIGHT} @ {FRAMERATE}fps...")
        video_config = picam2.create_video_configuration(
            main={"size": (WIDTH, HEIGHT)},
            controls={"FrameRate": FRAMERATE}
        )
        picam2.configure(video_config)

        # --- Encoder and Output Setup ---
        # The MJPEGEncoder will compress the raw frames from the camera
        encoder = MJPEGEncoder(bitrate=BITRATE)
        
        # The custom ZenohOutput will be the destination for the encoder's output
        zenoh_output = ZenohOutput(publisher, FRAME_ID, IMAGE_FORMAT)
        
        # `FileOutput` is a wrapper required by start_recording
        output = FileOutput(zenoh_output)

        # --- Start Streaming ---
        # `start_recording` runs in a background thread, continuously capturing,
        # encoding, and passing frames to our custom output's `write` method.
        picam2.start_recording(encoder, output)
        
        print(f"\nStreaming to Zenoh topic '{ZENOH_TOPIC}'...")
        print("Press Ctrl+C to stop.")
        
        # Keep the main thread alive while the background thread works
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nStopping stream...")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        # --- Cleanup ---
        print("Cleaning up resources.")
        if picam2.is_open:
            picam2.stop_recording()
            picam2.close()
            print("Camera closed.")
        
        if 'session' in locals():
            session.close()
            print("Zenoh session closed.")
