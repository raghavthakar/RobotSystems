from picarx_improved import Picarx, Sensor
from time import sleep
import readchar

if __name__ == "__main__":
    sensor = Sensor()
    while True:
        key = readchar.readkey()
        key = key.lower()

        if key == readchar.key.CTRL_C:
            print("\n Quit")
            break
        
        print(sensor.get_grayscale_data())
