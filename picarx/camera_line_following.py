import cv2
import numpy as np
from picarx_improved import Picarx

class LineFollower:
    def __init__(self, camera_index=0):
        self.camera = cv2.VideoCapture(camera_index)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.lines = None
        self.frame = None
        self.height = None
        self.width = None

    def sense(self):
        try:
            ret, self.frame = self.camera.read()
        except:
            return
        
        # Extract the bottom half of the frame
        self.height, self.width = self.frame.shape[:2]
        bottom_half = self.frame[self.height//2:self.height, :]

        gray = cv2.cvtColor(bottom_half, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)
        
        # Detect lines with less strict parameters
        self.lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=30, minLineLength=50, maxLineGap=20)
    
    def interpret(self):
        if self.lines is not None:
            # Initialize variables to store the sum of vector components
            sum_x = 0
            sum_y = 0
            
            for line in self.lines:
                x1, y1, x2, y2 = line[0]
                # Calculate the vector components of the line
                dx = x2 - x1
                dy = y2 - y1
                # Sum up the vector components
                sum_x += dx
                sum_y += dy
                
                # Adjust coordinates to the whole frame
                y1 += self.height // 2
                y2 += self.height // 2
                cv2.line(self.frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # Calculate the angle of the resultant vector
            angle = np.arctan2(sum_y, sum_x)
            
            # Normalize the angle to a range between -1 and 1
            normalized_angle = angle / np.pi
            
            # Print the normalized angle (value between -1 and 1)
            print("Normalized Angle:", normalized_angle)
            cv2.imshow('Line Detection', self.frame)

            return normalized_angle

    def cleanup(self):
        self.camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    px = Picarx()
    line_follower = LineFollower(camera_index=0)
    while True:
        line_follower.sense()
        pos = line_follower.interpret()
        try:
            px.forward(30)
            px.set_dir_servo_angle(30*pos)
        except:
            pass

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
    line_follower.cleanup()
