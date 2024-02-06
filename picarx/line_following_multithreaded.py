from picarx_improved import Picarx, Sensor, Interpreter, Controller, MessageBus
from time import sleep
import readchar
import concurrent.futures


if __name__ == "__main__":
    px = Picarx()
    sensor = Sensor()
    interpreter = Interpreter(True)
    controller = Controller(px)
    sensor_reading_bus = MessageBus()
    pos_bus = MessageBus()

    # sensor.produce(sensor_reading_bus, 0.1)
    # interpreter.consume_produce(sensor_reading_bus, pos_bus, 0.1)
    # controller.consume(pos_bus, 0.05)

    with concurrent.futures.ThreadPoolExecutor(max_workers=10) as executor:
        eSensor = executor.submit(sensor.produce, sensor_reading_bus, 0.1)
        eInterpreter = executor.submit(interpreter.consume_produce, sensor_reading_bus, pos_bus, 0.1)
        eController = executor.submit(controller.consume, pos_bus, 0.1)

    eSensor.result()
