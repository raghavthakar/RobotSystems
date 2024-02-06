from picarx_improved import Picarx, Sensor, Interpreter, Controller, MessageBus
from time import sleep
import readchar


if __name__ == "__main__":
    px = Picarx()
    sensor = Sensor()
    interpreter = Interpreter(True)
    controller = Controller()
    sensor_reading_bus = MessageBus()
    pos_bus = MessageBus()

    sensor.produce(sensor_reading_bus, 0.1)
    interpreter.consume_produce(sensor_reading_bus, pos_bus, 0.1)
    controller.consume(pos_bus, px, 0.05)
