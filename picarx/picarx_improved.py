import time
import os
import atexit
import logging
import math
from readerwriterlock import rwlock

try:
    from robot_hat import Pin, ADC, PWM, Servo, fileDB
    from robot_hat import Grayscale_Module, Ultrasonic
    from robot_hat.utils import reset_mcu, run_command
except ImportError:
    from sim_robot_hat import Pin, ADC, PWM, Servo, fileDB
    from sim_robot_hat import Grayscale_Module, Ultrasonic
    from sim_robot_hat import reset_mcu, run_command    
    print("Not on PiCar-X. Using simulated robot hat functions")

#Set Logging configuration
logging_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=logging_format, level=logging.INFO, datefmt="%H:%M:%S")
logging.getLogger().setLevel(logging.DEBUG)

# reset robot_hat
reset_mcu()
time.sleep(0.2)

def constrain(x, min_val, max_val):
    '''
    Constrains value to be within a range.
    '''
    return max(min_val, min(max_val, x))

class Picarx(object):
    CONFIG = '/opt/picar-x/picar-x.conf'

    DEFAULT_LINE_REF = [1000, 1000, 1000]
    DEFAULT_CLIFF_REF = [500, 500, 500]

    DIR_MIN = -30
    DIR_MAX = 30
    CAM_PAN_MIN = -90
    CAM_PAN_MAX = 90
    CAM_TILT_MIN = -35
    CAM_TILT_MAX = 65

    PERIOD = 4095
    PRESCALER = 10
    TIMEOUT = 0.02

    # servo_pins: camera_pan_servo, camera_tilt_servo, direction_servo
    # motor_pins: left_swicth, right_swicth, left_pwm, right_pwm
    # grayscale_pins: 3 adc channels
    # ultrasonic_pins: tring, echo2
    # config: path of config file
    def __init__(self, 
                servo_pins:list=['P0', 'P1', 'P2'], 
                motor_pins:list=['D4', 'D5', 'P12', 'P13'],
                grayscale_pins:list=['A0', 'A1', 'A2'],
                ultrasonic_pins:list=['D2','D3'],
                config:str=CONFIG,
                ):

        # --------- config_file ---------
        self.config_file = fileDB(config, 774, os.getlogin())

        # --------- servos init ---------
        self.cam_pan = Servo(servo_pins[0])
        self.cam_tilt = Servo(servo_pins[1])   
        self.dir_servo_pin = Servo(servo_pins[2])
        # get calibration values
        self.dir_cali_val = float(self.config_file.get("picarx_dir_servo", default_value=0))
        self.cam_pan_cali_val = float(self.config_file.get("picarx_cam_pan_servo", default_value=0))
        self.cam_tilt_cali_val = float(self.config_file.get("picarx_cam_tilt_servo", default_value=0))
        # set servos to init angle
        self.dir_servo_pin.angle(self.dir_cali_val)
        self.cam_pan.angle(self.cam_pan_cali_val)
        self.cam_tilt.angle(self.cam_tilt_cali_val)

        # --------- motors init ---------
        self.left_rear_dir_pin = Pin(motor_pins[0])
        self.right_rear_dir_pin = Pin(motor_pins[1])
        self.left_rear_pwm_pin = PWM(motor_pins[2])
        self.right_rear_pwm_pin = PWM(motor_pins[3])
        self.motor_direction_pins = [self.left_rear_dir_pin, self.right_rear_dir_pin]
        self.motor_speed_pins = [self.left_rear_pwm_pin, self.right_rear_pwm_pin]
        # get calibration values
        self.cali_dir_value = self.config_file.get("picarx_dir_motor", default_value="[1, 1]")
        self.cali_dir_value = [int(i.strip()) for i in self.cali_dir_value.strip().strip("[]").split(",")]
        self.cali_speed_value = [0, 0]
        self.dir_current_angle = 0
        self.dir_angle_value = 0 #cali+current angle
        # init pwm
        for pin in self.motor_speed_pins:
            pin.period(self.PERIOD)
            pin.prescaler(self.PRESCALER)

        # --------- ultrasonic init ---------
        tring, echo= ultrasonic_pins
        self.ultrasonic = Ultrasonic(Pin(tring), Pin(echo))

        # Register stop function to be executed at interpreter termination
        atexit.register(self.stop)
        
    def set_motor_speed(self, motor, speed):
        ''' set motor speed
        
        param motor: motor index, 1 means left motor, 2 means right motor
        type motor: int
        param speed: speed
        type speed: int      
        '''
        speed = constrain(speed, -100, 100)
        motor -= 1
        if speed >= 0:
            direction = 1 * self.cali_dir_value[motor]
        elif speed < 0:
            direction = -1 * self.cali_dir_value[motor]
        speed = abs(speed)
        # Linear scaling removed
        # if speed != 0:
        #     speed = int(speed /2 ) + 50
        speed = speed - self.cali_speed_value[motor]
        if direction < 0:
            self.motor_direction_pins[motor].high()
            self.motor_speed_pins[motor].pulse_width_percent(speed)
        else:
            self.motor_direction_pins[motor].low()
            self.motor_speed_pins[motor].pulse_width_percent(speed)
        
        logging.debug(f"set_motor_speed: {speed}")

    def motor_speed_calibration(self, value):
        self.cali_speed_value = value
        if value < 0:
            self.cali_speed_value[0] = 0
            self.cali_speed_value[1] = abs(self.cali_speed_value)
        else:
            self.cali_speed_value[0] = abs(self.cali_speed_value)
            self.cali_speed_value[1] = 0

    def motor_direction_calibrate(self, motor, value):
        ''' set motor direction calibration value
        
        param motor: motor index, 1 means left motor, 2 means right motor
        type motor: int
        param value: speed
        type value: int
        '''      
        motor -= 1
        if value == 1:
            self.cali_dir_value[motor] = 1
        elif value == -1:
            self.cali_dir_value[motor] = -1
        self.config_file.set("picarx_dir_motor", self.cali_dir_value)

    def dir_servo_calibrate(self, value):
        self.dir_cali_val = value
        self.config_file.set("picarx_dir_servo", "%s"%value)
        self.dir_servo_pin.angle(value)

    def set_dir_servo_angle(self, value):
        self.dir_current_angle = constrain(value, self.DIR_MIN, self.DIR_MAX)
        self.dir_angle_value  = self.dir_current_angle + self.dir_cali_val
        self.dir_servo_pin.angle(self.dir_angle_value)

        logging.debug(f"set_dir_servo_angle: {self.dir_angle_value}")

    def cam_pan_servo_calibrate(self, value):
        self.cam_pan_cali_val = value
        self.config_file.set("picarx_cam_pan_servo", "%s"%value)
        self.cam_pan.angle(value)

    def cam_tilt_servo_calibrate(self, value):
        self.cam_tilt_cali_val = value
        self.config_file.set("picarx_cam_tilt_servo", "%s"%value)
        self.cam_tilt.angle(value)

    def set_cam_pan_angle(self, value):
        value = constrain(value, self.CAM_PAN_MIN, self.CAM_PAN_MAX)
        self.cam_pan.angle(-1*(value + -1*self.cam_pan_cali_val))

    def set_cam_tilt_angle(self,value):
        value = constrain(value, self.CAM_TILT_MIN, self.CAM_TILT_MAX)
        self.cam_tilt.angle(-1*(value + -1*self.cam_tilt_cali_val))

    def set_power(self, speed):
        self.set_motor_speed(1, speed)
        self.set_motor_speed(2, speed)

    def get_ackermann_scaling_factor(self):
        l = 0.1
        w = 0.12

        #Calculate inner and outer angles
        inner_angle = math.atan((2*l*math.sin(self.dir_angle_value))/(2*l*math.cos(self.dir_angle_value) - w*math.sin(self.dir_angle_value)))
        outer_angle = math.atan((2*l*math.sin(self.dir_angle_value))/(2*l*math.cos(self.dir_angle_value) + w*math.sin(self.dir_angle_value)))
        
        #If angles are zero set scaling to 1
        if (inner_angle == 0) or (outer_angle == 0):
            scaling = 1
        else:
            abs_outer_angle = abs(outer_angle)
            abs_inner_angle = abs(inner_angle)

            if abs(outer_angle) > abs(inner_angle):
                scaling = abs_inner_angle/abs_outer_angle
            else:
                scaling = abs_outer_angle/abs_inner_angle
        
        return scaling

    def backward(self, speed):
        current_angle = self.dir_current_angle
        if current_angle != 0:
            abs_current_angle = abs(current_angle)
            if abs_current_angle > self.DIR_MAX:
                abs_current_angle = self.DIR_MAX

            # power_scale = (100 - abs_current_angle) / 100.0
            ackermann_scaling_factor = self.get_ackermann_scaling_factor()

            if (current_angle / abs_current_angle) > 0:
                self.set_motor_speed(1, -1*speed)
                self.set_motor_speed(2, speed * ackermann_scaling_factor)
            else:
                self.set_motor_speed(1, -1*speed * ackermann_scaling_factor)
                self.set_motor_speed(2, speed )
        else:
            self.set_motor_speed(1, -1*speed)
            self.set_motor_speed(2, speed)  

    def forward(self, speed):
        current_angle = self.dir_current_angle
        if current_angle != 0:
            abs_current_angle = abs(current_angle)
            if abs_current_angle > self.DIR_MAX:
                abs_current_angle = self.DIR_MAX
            
            # power_scale = (100 - abs_current_angle) / 100.0
            ackermann_scaling_factor = self.get_ackermann_scaling_factor()
            
            if (current_angle / abs_current_angle) > 0:
                self.set_motor_speed(1, 1*speed * ackermann_scaling_factor)
                self.set_motor_speed(2, -speed) 
            else:
                self.set_motor_speed(1, speed)
                self.set_motor_speed(2, -1*speed * ackermann_scaling_factor)
        else:
            self.set_motor_speed(1, speed)
            self.set_motor_speed(2, -1*speed)                  

    def stop(self):
        '''
        Execute twice to make sure it stops
        '''
        for _ in range(2):
            self.motor_speed_pins[0].pulse_width_percent(0)
            self.motor_speed_pins[1].pulse_width_percent(0)
            time.sleep(0.002)

    def get_distance(self):
        return self.ultrasonic.read()
        
class Sensor():
    CONFIG = '/opt/picar-x/picar-x.conf'

    DEFAULT_LINE_REF = [1000, 1000, 1000]
    DEFAULT_CLIFF_REF = [500, 500, 500]

    DIR_MIN = -30
    DIR_MAX = 30
    CAM_PAN_MIN = -90
    CAM_PAN_MAX = 90
    CAM_TILT_MIN = -35
    CAM_TILT_MAX = 65

    PERIOD = 4095
    PRESCALER = 10
    TIMEOUT = 0.02

    def __init__(self, grayscale_pins:list=['A0', 'A1', 'A2']) -> None:
        # --------- config_file ---------
        self.config_file = fileDB(self.CONFIG, 774, os.getlogin())

        # --------- grayscale module init ---------
        adc0, adc1, adc2 = [ADC(pin) for pin in grayscale_pins]
        self.grayscale = Grayscale_Module(adc0, adc1, adc2, reference=None)
        # get reference
        self.line_reference = self.config_file.get("line_reference", default_value=str(self.DEFAULT_LINE_REF))
        self.line_reference = [float(i) for i in self.line_reference.strip().strip('[]').split(',')]
        self.cliff_reference = self.config_file.get("cliff_reference", default_value=str(self.DEFAULT_CLIFF_REF))
        self.cliff_reference = [float(i) for i in self.cliff_reference.strip().strip('[]').split(',')]
        # transfer reference
        self.grayscale.reference(self.line_reference)

    def set_grayscale_reference(self, value):
        if isinstance(value, list) and len(value) == 3:
            self.line_reference = value
            self.grayscale.reference(self.line_reference)
            self.config_file.set("line_reference", self.line_reference)
        else:
            raise ValueError("grayscale reference must be a 1*3 list")

    def get_grayscale_data(self):
        return list.copy(self.grayscale.read())

    def get_line_status(self,gm_val_list):
        return self.grayscale.read_status(gm_val_list)

    def set_line_reference(self, value):
        self.set_grayscale_reference(value)

    def get_cliff_status(self,gm_val_list):
        for i in range(0,3):
            if gm_val_list[i]<=self.cliff_reference[i]:
                return True
        return False

    def set_cliff_reference(self, value):
        if isinstance(value, list) and len(value) == 3:
            self.cliff_reference = value
            self.config_file.set("cliff_reference", self.cliff_reference)
        else:
            raise ValueError("grayscale reference must be a 1*3 list")

    def produce(self, writebus, delay_time):
        while True:
            writebus.write(self.grayscale.read())
            time.sleep(delay_time)

class Interpreter():
    def __init__(self, line_is_dark=True, dark_light_diff=75) -> None:
        # LIGHT = higher value
        # DARK  = lower  value
        self.line_is_dark_factor = 1 if line_is_dark else -1
        self.dark_light_diff = dark_light_diff
        self.prev_line_relative_pos = 0
    
    def round_to_nearest(self, value):
        diff_a = abs(value - -self.dark_light_diff)
        diff_b = abs(value - 0)
        diff_c = abs(value - self.dark_light_diff)
        
        min_diff = min(diff_a, diff_b, diff_c)
        
        if min_diff == diff_a:
            return -self.dark_light_diff
        elif min_diff == diff_b:
            return 0
        else:
            return self.dark_light_diff

    def get_line_relative_pos(self, sensor_reading):

        del_lc = sensor_reading[0] - sensor_reading[1]
        del_cr = sensor_reading[1] - sensor_reading[2]
        del_rl = sensor_reading[2] - sensor_reading[0]

        del_lc = self.round_to_nearest(del_lc)
        del_cr = self.round_to_nearest(del_cr)
        del_rl = self.round_to_nearest(del_rl)

        print(del_lc, del_cr, del_rl)

        line_relative_pos = 0

        if del_lc == 0 and del_cr == -self.dark_light_diff and del_rl == self.dark_light_diff:
            line_relative_pos = -0.5 * self.line_is_dark_factor
        elif del_lc == -self.dark_light_diff and del_cr == 0 and del_rl == self.dark_light_diff:
            line_relative_pos = -1 * self.line_is_dark_factor
        elif del_lc == self.dark_light_diff and del_cr == 0 and del_rl == -self.dark_light_diff:
            line_relative_pos = 0.5 * self.line_is_dark_factor
        elif del_lc == self.dark_light_diff and del_cr == -self.dark_light_diff and del_rl == 0:
            line_relative_pos = 0 * self.line_is_dark_factor
        elif del_lc == 0 and del_cr == self.dark_light_diff and del_rl == -self.dark_light_diff:
            line_relative_pos = 1 * self.line_is_dark_factor
        else:
            line_relative_pos = self.prev_line_relative_pos
        
        self.prev_line_relative_pos = line_relative_pos
        return line_relative_pos
     
    def consume_produce(self, readbus, writebus, delay_time):
        while True:
            line_relative_pos = self.get_line_relative_pos(readbus.read())
            writebus.write(line_relative_pos)
            time.sleep(delay_time)

class Controller():
    def __init__(self, px, scale_factor = 15) -> None:
        self.scale_factor = scale_factor
        self.px = px
    
    def control(self, px, pos):
        px.set_dir_servo_angle(self.scale_factor*pos)
    
    def consume(self, readbus, delay_time):
        while True:
            pos = readbus.read()
            try:
                self.px.forward(25)
                self.px.set_dir_servo_angle(35*pos)
            except:
                pass
            time.sleep(delay_time)

class MessageBus():
    def __init__(self) -> None:
        self.message = None
        self.lock = rwlock.RWLockWriteD()
    def write(self, message):
        with self.lock.gen_wlock():
            self.message = message
    def read(self):
        with self.lock.gen_rlock():
            message = self.message
        return message

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
            return 1
        return 0

class ultra_controller():
    def __init__(self, car=None, speed=55):
        self.car = car
        self.speed=speed
    
    def control(self, cmd):
        if cmd:
            print('Forward')
            self.car.forward(self.speed+10)
            time.sleep(0.1) # TODO <- this a problem??
            self.car.forward(self.speed)
        else:
            print('Stop')
            self.car.stop()
     

if __name__ == "__main__":
    px = Picarx()
    px.forward(50)
    time.sleep(1)
    px.stop()   
