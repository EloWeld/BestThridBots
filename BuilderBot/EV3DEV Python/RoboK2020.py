#!/usr/bin/env python3
import time

from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D, MoveSteering, MoveTank
from ev3dev2.sensor.lego import ColorSensor
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.led import Leds
from ev3dev2.sound import Sound
from ev3dev2.button import Button

# ------------------------------PINS------------------------------
pin_motors = {'LEFT': OUTPUT_A,
              'RIGHT': OUTPUT_D}

sens_line = ColorSensor(INPUT_1)
sens_cross = ColorSensor(INPUT_4)

# ------------------------------CONSTANS------------------------------
# Movement
wheel_radius = 55 // 2
rotate_radius = 150 // 2
motor_speed = -60
cross_center_dist = 120
dist_to_cube = 335 - cross_center_dist

# Color
white_val = 66
black_val = 10
middle_val = (white_val + black_val) / 2 - 10  # Not middle yet

# PID
kp = 0.9


# ------------------------------DEBUG------------------------------
class Debug:
    @staticmethod
    def beep():
        Sound().beep()

    @staticmethod
    def say(text='Welcome to DEBUG!'):
        Sound().speak(text, volume=80)

    @staticmethod
    def blink(clr_l, clr_r):
        Leds().set_color('LEFT', clr_l)
        Leds().set_color('RIGHT', clr_r)

    def wait_for_key(self):
        self.blink('AMBER', 'AMBER')
        Button().wait_for_bump('enter')
        self.blink('GREEN', 'GREEN')


# ------------------------------VOID------------------------------


# ---------MOTORS-------------
class MotorsController:
    def __init__(self):
        self.motor_left = LargeMotor(pin_motors['LEFT'])
        self.motor_right = LargeMotor(pin_motors['RIGHT'])
        self.motors_s = MoveSteering(pin_motors['LEFT'], pin_motors['RIGHT'])
        self.motors_t = MoveTank(pin_motors['LEFT'], pin_motors['RIGHT'])

    def get_motor_enc(self, motor='LEFT'):
        if motor == 'LEFT':
            return self.motor_left.position_sp
        else:
            return self.motor_right.position_sp

    def rotate_90_left(self):
        angle = 90 * rotate_radius // wheel_radius  # 2*pi*R * 1/4 // 2 * pi* r // 90
        self.motors_s.on_for_degrees(steering=-100, speed=motor_speed, degrees=angle)

    def rotate_90_right(self):
        angle = 90 * rotate_radius // wheel_radius
        self.motors_s.on_for_degrees(steering=100, speed=motor_speed, degrees=angle)

    def rotate_180_left(self):
        angle = 180 * rotate_radius // wheel_radius
        self.motors_s.on_for_degrees(steering=-100, speed=motor_speed, degrees=angle)

    def rotate_180_right(self):
        angle = 180 * rotate_radius // wheel_radius
        self.motors_s.on_for_degrees(steering=100, speed=motor_speed, degrees=angle)

    def move_to_cross_center(self):
        angle = 180 * cross_center_dist // wheel_radius * 3.14
        self.motors_s.on_for_degrees(steering=0, speed=motor_speed, degrees=angle)


def line_move(mc, distantion=0):
    # Save initially values on encoders(in degrees)
    motor_l_enc = mc.get_motor_enc('LEFT')
    motor_r_enc = mc.get_motor_enc('RIGHT')
    # Cross or distantion
    condition = (sens_cross.reflected_light_intensity > middle_val) if distantion == 0 else \
        (mc.get_motor_enc('LEFT') - motor_l_enc < distantion and mc.get_motor_enc('RIGHT') - motor_r_enc < distantion)
    while condition:
        # Calculate PID(P) value
        p_val = (sens_line.reflected_light_intensity - middle_val - middle_val) * kp
        # Run motors with speed by PID(P) value
        speed_l = p_val if p_val < 0 else -p_val
        mc.motors_t.on(speed_l, motor_speed)
    # Stop motors after cross/distantion_limit
    mc.motors_t.stop()


# ------------------------------MAIN------------------------------
def setup():
    Debug.blink('GREEN', 'GREEN')


def main():
    setup()
    mc = MotorsController()

    line_move(mc)
    Debug.beep()
    Button().wait_for_bump(['enter'])
    mc.rotate_180_right()
    Debug.beep()
    Button().wait_for_bump(['enter'])
    mc.rotate_90_left()

    Debug.beep()
    while Button().buttons_pressed != ['up']:
        Sound().play_file('/home/robot/troll.wav', 70)
        print('HA-HA!')


if __name__ == '__main__':
    while True:
        Debug.blink('RED', 'RED')
        Button().wait_for_bump('enter')
        main()