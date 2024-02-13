#import time
from time import sleep, time
import picarx_improved as pc
import rossros as rr

try:
    from robot_hat import Pin, ADC, PWM, Servo, fileDB
    from robot_hat import Ultrasonic
    from robot_hat.utils import reset_mcu
except ImportError:
    print('ImportError, using sim_robot_hat')
    from sim_robot_hat import Pin, ADC, PWM, Servo, fileDB
    from sim_robot_hat import Ultrasonic
    from sim_robot_hat import reset_mcu

reset_mcu()
sleep(0.2)

class gray_sensor():

    def __init__(self, grayscale_pins:list=['A0', 'A1', 'A2']):
        # set up adc structures
        self.adc0, self.adc1, self.adc2 = [ADC(pin) for pin in grayscale_pins]
        #self.grayscale = Grayscale_Module(adc0, adc1, adc2, reference=None)

    def sensor_reading(self):
        # poll the 3 ADC structures & compile ouputs into list
        adc_value_list = []
        adc_value_list.append(self.adc0.read())
        adc_value_list.append(self.adc1.read())
        adc_value_list.append(self.adc2.read())
        print('GR_READING:', adc_value_list)     
        return list.copy(adc_value_list)

class gray_interpreter():

    def __init__(self, sensitivity=0.1, polarity='d'):
        # Take arguments for sensitivity (how different dark and light
        # readings are expected to be) and polarity (whether line is 
        # darker or lighter than floor)
        self.sensitivity = sensitivity
        self.polarity = polarity
        self.running_avg = [[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]]

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
        
        print('GR_RUNNING_READINGS:', self.running_avg)
        averaged_rdgs = [sum(v)/3 for v in self.running_avg]        
        min_v = min(averaged_rdgs)
        max_v = max(averaged_rdgs)
        
        try:
            norm_rdgs = [2*((v-min_v)/(max_v-min_v))-1 for v in averaged_rdgs]
        except:
            print('Norm error in grayscale interp')
            norm_rdgs = [0.0, 0.0, 0.0]
            
        if self.polarity == 'l':
            norm_rdgs = [(-1)*val for val in norm_rdgs]
            
        #print('Normalized Rdgs:', norm_rdgs)
        adjusted_readings = norm_rdgs
        
        position = self.output(adjusted_readings)
        print('GR_INTERPRET:', position)
        return position

    def output(self, adjusted_readings):
        # Return position of robot relative to line as a value on interval
        # [-1,1], with positive values being left of robot.
        if adjusted_readings == None: return 0
        left = adjusted_readings[0]
        mid = adjusted_readings[1]
        right = adjusted_readings[2]

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

class gray_controller():

    def __init__(self, car=None, scaling_factor=30):
        # take in an argument (with a default value) for the scaling factor
        # between the interpreted offset from the line and the angle by which 
        # to steer
        self.car = car
        self.scaling_factor = scaling_factor

    def control(self, rel_pos=0.0):
        # main control method should call the steering-servo method from your 
        # car class so that it turns the car toward the line. It should also 
        # return the commanded steering angle.
        val = int(rel_pos * self.scaling_factor)
        print('GR_CONTROL: ', val)
        self.car.set_dir_servo_angle(val)
            
class ultra_sensor():
    def __init__(self, trig=Pin('D2'), echo=Pin('D3')):
        self.ultrasonic = Ultrasonic(trig, echo)
        
    def sensor_reading(self):
        dist = self.ultrasonic.read()
        return dist

class ultra_interpreter():
    def __init__(self, stop_dist = 5):
        self.stop_dist = stop_dist
        self.running_rdgs = [0.0,0.0,0.0]
        
    # return true if car should go forward
    def process_reading(self, reading):
        self.running_rdgs[0] = self.running_rdgs[1]
        self.running_rdgs[1] = self.running_rdgs[2]
        self.running_rdgs[2] = reading
        avg_rdg = sum(self.running_rdgs) / 3
        if avg_rdg > self.stop_dist:
            return True
        return False

class ultra_controller():
    def __init__(self, car=None, speed=55):
        self.car = car
        self.speed=speed
    
    def control(self, cmd):
        if cmd:
            print('Forward')
            self.car.forward(self.speed+10)
            sleep(0.1) # TODO <- this a problem??
            self.car.forward(self.speed)
        else:
            print('Stop')
            self.car.stop()
     
def robot_hat_reset(bus=None):
    print('RESET')
    reset_mcu()
    sleep(0.2)   

pol = str(input('Enter L (lighter) or D (darker) polarity: ')).lower()

picar = pc.Picarx()
gray_sense = gray_sensor()
gray_interpret = gray_interpreter(polarity=pol)
steer_control = gray_controller(car=picar)
ultra_sense = ultra_sensor()
ultra_interpret = ultra_interpreter(stop_dist=15)
drive_control = ultra_controller(car=picar, speed=55)
print('SIC functions defined')

gr_sense_bus = rr.Bus(name="Greyscale bus")
gr_processed_bus = rr.Bus(name="Steer Val bus")
ut_sense_bus = rr.Bus(name='Ultrasonic bus')
ut_processed_bus = rr.Bus(name='Drive val bus')
term_bus = rr.Bus(False, name="Termination bus")
print('Busses defined')

# Syncing these seems to help
gr_sense_delay = 0.25 # 0.025
gr_interpret_delay = 0.25 # 0.025
steer_delay = 0.25 # 0.25 # 

ut_sense_delay = 0.25
ut_interpret_delay = 0.25
drive_delay = 0.25

time_delay = 0.25
reset_delay = 2.5
RUN_TIME = 5

prod_con_list = []
print_list = []

'''
res = rr.Consumer(consumer_function=robot_hat_reset,
                  input_buses=None,
                  delay=reset_delay,
                  termination_buses=term_bus)
prod_con_list.append(res)
'''

gr_prod = rr.Producer(producer_function=gray_sense.sensor_reading,
                   output_buses=gr_sense_bus,
                   delay=gr_sense_delay,
                   termination_buses=term_bus
                   )
prod_con_list.append(gr_prod)
print_list.append(gr_prod)


ut_prod = rr.Producer(producer_function=ultra_sense.sensor_reading,
                   output_buses=ut_sense_bus,
                   delay=ut_sense_delay,
                   termination_buses=term_bus
                   )
prod_con_list.append(ut_prod)
print_list.append(ut_prod)

print('Producers defined')

gr_interp = rr.ConsumerProducer(consumer_producer_function=gray_interpret.process_reading,
                             input_buses=gr_sense_bus,
                             output_buses=gr_processed_bus,
                             delay=gr_interpret_delay,
                             termination_buses=term_bus
                             )
prod_con_list.append(gr_interp)
print_list.append(gr_interp)

ut_interp = rr.ConsumerProducer(consumer_producer_function=ultra_interpret.process_reading,
                             input_buses=ut_sense_bus,
                             output_buses=ut_processed_bus,
                             delay=ut_interpret_delay,
                             termination_buses=term_bus
                             )
prod_con_list.append(ut_interp)
print_list.append(ut_interp)

print('Interpreters defined')

gr_cons = rr.Consumer(consumer_function=steer_control.control,
                   input_buses=gr_processed_bus,
                   delay=steer_delay,
                   termination_buses=term_bus
                   )
prod_con_list.append(gr_cons)
print_list.append(gr_cons)

ut_cons = rr.Consumer(consumer_function=drive_control.control,
                   input_buses=ut_processed_bus,
                   delay=drive_delay,
                   termination_buses=term_bus
                   )
prod_con_list.append(ut_cons)
print_list.append(ut_cons)

print('Consumers defined')

time = rr.Timer(output_buses=term_bus,
                duration=RUN_TIME,
                delay=time_delay,
                termination_buses=term_bus
                )
prod_con_list.append(time)
print('Timer defined')

print('Setup complete, running concurrently')

rr.runConcurrently(prod_con_list)