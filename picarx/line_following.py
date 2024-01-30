from picarx_improved import Picarx, Sensor, Interpreter, Controller
from time import sleep
import readchar

if __name__ == "__main__":
    px = Picarx()
    sensor = Sensor()
    interpreter = Interpreter()
    controller = Controller

    while True:
        pos = (interpreter.get_line_relative_pos(sensor.grayscale.read()))
        controller.control(px, pos)
        sleep(0.1)