from picarx_improved import Picarx
from time import sleep
import readchar

manual = '''
Press keys on keyboard to control PiCar-X!
    q: Forward and left
    w: Forward
    e: Forward and right
    a: Backward and left
    s: Backward
    d: Backward and rifht
    ctrl+c: Press twice to exit the program
'''


if __name__ == "__main__":
    try:
        pan_angle = 0
        tilt_angle = 0
        px = Picarx()
        print(manual)
        
        while True:
            key = readchar.readkey()
            key = key.lower()
            if key in('qweasd'):
                print('Performing controlled movement', end=' ')
                if 'q' == key:
                    print('forward and left.')
                    px.set_dir_servo_angle(-15)
                    px.forward(50)
                elif 'w' == key:
                    print('forward.')
                    px.set_dir_servo_angle(0)
                    px.forward(50)
                elif 'e' == key:
                    print('forward and right.')
                    px.set_dir_servo_angle(15)
                    px.forward(50)
                elif 'a' == key:
                    print('backward and left.')
                    px.set_dir_servo_angle(-15)
                    px.backward(50)
                elif 's' == key:
                    print('backward.')
                    px.set_dir_servo_angle(0)
                    px.backward(50)
                elif 'd' == key:
                    print('backward and right.')
                    px.set_dir_servo_angle(15)
                    px.backward(50)
                sleep(0.5)
                px.forward(0)
          
            elif key == readchar.key.CTRL_C:
                print("\n Quit")
                break

    finally:
        px.set_cam_tilt_angle(0)
        px.set_cam_pan_angle(0)  
        px.set_dir_servo_angle(0)  
        px.stop()
        sleep(.2)


