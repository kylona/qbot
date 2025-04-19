from picamera2 import Picamera2
import time

picam2 = Picamera2()
config = picam2.create_still_configuration()
picam2.configure(config)

picam2.start()

for n in range(20):
    print("Capturing in . . .")
    for i in range(3):
        print(3-i)
        time.sleep(1)
    print("Smile!")

    picam2.capture_file(f"calib_images/checkerboard_{n}.png")
    print(f"Saved as: checkerboard_{n}.png")
    print("Press enter to take next photo")
    input()

picam2.stop()
