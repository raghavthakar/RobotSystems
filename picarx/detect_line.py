import cv2
import numpy as np

class LineFollower:
    def __init__(self, camera_index=0):
        self.camera = cv2.VideoCapture(camera_index)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    def detect_lines(self):
        while True:
            ret, frame = self.camera.read()
            if not ret:
                break
            
            # Extract the bottom half of the frame
            height, width = frame.shape[:2]
            bottom_half = frame[height//2:height, :]

            gray = cv2.cvtColor(bottom_half, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            edges = cv2.Canny(blurred, 50, 150)
            
            # Detect lines with less strict parameters
            lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=30, minLineLength=50, maxLineGap=20)
            
            if lines is not None:
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    # Adjust coordinates to the whole frame
                    y1 += height // 2
                    y2 += height // 2
                    cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            cv2.imshow('Line Detection', frame)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

    def cleanup(self):
        self.camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    line_follower = LineFollower(camera_index=0)
    line_follower.detect_lines()
    line_follower.cleanup()
