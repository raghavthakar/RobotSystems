from picarx_improved import Picarx, Sensor
from time import sleep
import readchar

if __name__ == "__main__":
    sensor = Sensor()
    print(sensor.line_calibration())
