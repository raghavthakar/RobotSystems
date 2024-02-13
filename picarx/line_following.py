from picarx_improved import Picarx, Sensor, Interpreter, Controller
from time import sleep
import readchar


if __name__ == "__main__":
    px = Picarx()
    sensor = Sensor()
    interpreter = Interpreter(False)
    controller = Controller(px)

    while True:
        pos = interpreter.get_line_relative_pos(sensor.grayscale.read())
        try:
            px.forward(25)
            px.set_dir_servo_angle(40*pos)
        except:
            pass
        sleep(0.1)
