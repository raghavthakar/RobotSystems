from picarx_improved import Picarx, Sensor, Interpreter
from time import sleep
import readchar

if __name__ == "__main__":
    sensor = Sensor()
    interpreter = Interpreter()

    while True:
        print(interpreter.get_line_relative_pos(sensor.grayscale.read()))