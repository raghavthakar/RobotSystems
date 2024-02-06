from time import sleep
from picarx_improved import Picarx, MessageBus as bus
import concurrent.futures

try:
    from robot_hat import Pin, ADC, PWM, Servo, fileDB
    from robot_hat import Grayscale_Module
    from robot_hat.utils import reset_mcu
except ImportError:
    print('ImportError, using sim_robot_hat')
    from sim_robot_hat import Pin, ADC, PWM, Servo, fileDB
    from sim_robot_hat import Grayscale_Module
    from sim_robot_hat import reset_mcu

reset_mcu()
sleep(0.2)

class sensor():

    def __init__(self, grayscale_pins:list=['A0', 'A1', 'A2']):
        # set up adc structures
        self.adc0, self.adc1, self.adc2 = [ADC(pin) for pin in grayscale_pins]
        #self.grayscale = Grayscale_Module(adc0, adc1, adc2, reference=None)
        self.adc_value_list = []

    def sensor_reading(self):
        # poll the 3 ADC structures & compile ouputs into list
        self.adc_value_list = []
        self.adc_value_list.append(self.adc0.read())
        self.adc_value_list.append(self.adc1.read())
        self.adc_value_list.append(self.adc2.read())
    
    def producer(self, bus, delay):
        while True:
            self.sensor_reading()
            bus.write(self.adc_value_list)
            #print('Sensed Vals:',self.adc_value_list)
            sleep(delay)
        

class interpreter():

    def __init__(self, sensitivity=0.1, polarity='d'):
        # Take arguments for sensitivity (how different dark and light
        # readings are expected to be) and polarity (whether line is 
        # darker or lighter than floor)
        self.sensitivity = sensitivity
        self.polarity = polarity
        self.running_avg = [[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]]
        self.adjusted_readings = []

    def process_reading(self, reading):
        # Take input of same format as output of sensor method. Then,
        # identify whether there is a sharp change between 2 adjacent
        # sensor vals (edge), than using edge location & sign, determine
        # whether system is left or right of centered, and magnitude of
        # misalignment.

        # Function should be robust to lighting conditions.
        # Function should have option to have the "target" (line) darker
        # or lighter than surrounding floor.
        for i in range(3):
            self.running_avg[i][0] = self.running_avg[i][1]
            self.running_avg[i][1] = self.running_avg[i][2]
            self.running_avg[i][2] = reading[i]
        
        averaged_rdgs = [sum(v)/3 for v in self.running_avg]        
        min_v = min(averaged_rdgs)
        max_v = max(averaged_rdgs)
        norm_rdgs = [2*((v-min_v)/(max_v-min_v))-1 for v in averaged_rdgs]
        
        if self.polarity == 'l':
            norm_rdgs = [(-1)*val for val in norm_rdgs]

        self.adjusted_readings = norm_rdgs

    def output(self):
        # Return position of robot relative to line as a value on interval
        # [-1,1], with positive values being left of robot.
        if self.adjusted_readings == None: return 0
        left = self.adjusted_readings[0]
        mid = self.adjusted_readings[1]
        right = self.adjusted_readings[2]

        position = 0.0
        magnitude = abs(left-right)

        # If large difference between outside sensors, process steering
        if magnitude > self.sensitivity:
            left_diff = abs(mid - left)
            right_diff = abs(mid - right)
            # if a difference is small on left and left val > right val, steer left TODO - may not want >
            if left_diff < right_diff and left > right:
                # turn left
                position = magnitude
            elif left_diff < right_diff and left < right:
                # turn right
                position = -1*magnitude
            elif left_diff > right_diff and left > right:
                # turn left
                position = magnitude
            elif left_diff > right_diff and left < right:
                # turn right
                position = -1*magnitude

        return position
    
    def consumer_producer(self, read_bus, write_bus, delay):
        while True:
            reading = read_bus.read()
            self.process_reading(reading)
            position = self.output()
            write_bus.write(position)
            sleep(delay)

class controller():

    def __init__(self, scaling_factor=30):
        # take in an argument (with a default value) for the scaling factor
        # between the interpreted offset from the line and the angle by which 
        # to steer
        self.scaling_factor = scaling_factor

    def control(self, rel_pos=0.0):
        # main control method should call the steering-servo method from your 
        # car class so that it turns the car toward the line. It should also 
        # return the commanded steering angle.
        return int(rel_pos * self.scaling_factor)
    
    def consumer(self, bus, car, delay):
        while True:
            rel_pos = bus.read()
            angle = self.control(rel_pos)
            car.set_dir_servo_angle(angle)
            print('Processed Val:', rel_pos,'| Commanded Angle:', angle)
            sleep(delay)
            

pol = str(input('Enter L (lighter) or D (darker) polarity: ')).lower()
sense = sensor()
interpret = interpreter(polarity=pol)
control = controller()
car = Picarx()
sense_bus = bus.bus()
processed_bus = bus.bus()

sense_delay = 0.1
control_delay = 0.25

car.forward(65)
sleep(0.25)
car.forward(55)

with concurrent.futures.ThreadPoolExecutor(max_workers=3) as executor:
    eSensor = executor.submit(sense.producer, sense_bus, sense_delay)
    eInterpreter = executor.submit(interpret.consumer_producer, sense_bus, processed_bus, sense_delay)
    eController = executor.submit(control.consumer, processed_bus, car, control_delay)
    
eSensor.result()
eInterpreter.result()
eController.result()