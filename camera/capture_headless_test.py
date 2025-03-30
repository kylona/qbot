from picamera2 import Picamera2
import time

picam2 = Picamera2()
config = picam2.create_still_configuration()
picam2.configure(config)

picam2.start()

while True:
    picam2.capture_file("demo.jpg")
    picam2.set_controls({"AfMode": 1, "AfTrigger": 0})
    time.sleep(1.0)

picam2.stop()
