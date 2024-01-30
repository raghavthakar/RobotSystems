import picamera
import time
import io
import cv2
import numpy as np

# Create a stream to hold the image data
stream = io.BytesIO()

# Initialize the camera
with picamera.PiCamera() as camera:
    camera.resolution = (640, 480)  # Set the resolution
    camera.framerate = 24            # Set the framerate

    # Start a preview on the Pi's screen
    camera.start_preview()

    # Wait for the camera to warm up
    time.sleep(2)

    try:
        for _ in camera.capture_continuous(stream, format='jpeg', use_video_port=True):
            # Rewind the stream to read the image data from the beginning
            stream.seek(0)

            # Convert image data into a numpy array
            data = np.frombuffer(stream.getvalue(), dtype=np.uint8)

            # Decode the numpy array into an OpenCV image
            image = cv2.imdecode(data, 1)

            # Display the image
            cv2.imshow('Camera Stream', image)
            cv2.waitKey(1)  # Adjust delay as needed

            # Clear the stream in preparation for the next frame
            stream.seek(0)
            stream.truncate()

    finally:
        cv2.destroyAllWindows()

