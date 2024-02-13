from picarx_improved import Picarx, Sensor, Interpreter, Controller, ultra_sensor, ultra_interpreter
from time import sleep, time
import readchar



if __name__ == "__main__":
    duration = 15 # How long to run for
    start_time = time()
    
    px = Picarx()
    sensor = Sensor()
    interpreter = Interpreter(False)
    controller = Controller(px)

    ultra_sense = ultra_sensor()
    ultra_interpret = ultra_interpreter(stop_dist=15)

    while (time() - start_time) < duration:
        pos = interpreter.get_line_relative_pos(sensor.grayscale.read())
        try:
            px.forward(25 * ultra_interpret.process_reading())
            px.set_dir_servo_angle(30 * pos)
        except Exception as e:
            print("An error occurred:", e)
        sleep(0.1)
