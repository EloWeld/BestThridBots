#!/usr/bin/env python3
import time

from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D, MoveSteering, MoveTank
from ev3dev2.sensor.lego import ColorSensor
from ev3dev2.sensor import INPUT_4, INPUT_1
from ev3dev2.led import Leds
from ev3dev2.sound import Sound
from ev3dev2.button import Button

# ------------------------------PINS------------------------------
motor_l = LargeMotor(OUTPUT_A)
motor_r = LargeMotor(OUTPUT_D)
motors = MoveSteering(OUTPUT_D, OUTPUT_A)
motors_t = MoveTank(OUTPUT_D, OUTPUT_A)

sens_line = ColorSensor(INPUT_1)
sens_cross = ColorSensor(INPUT_4)

# ------------------------------CONSTANS------------------------------
# Movement
wheel_radius = 55 // 2
rotate_radius = 160 // 2
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
def beep():
    Sound().beep()


def say(text='Welcome to DEBUG!'):
    Sound().speak(text, volume=80)


def blink(clr_l, clr_r):
    Leds().set_color('LEFT', clr_l)
    Leds().set_color('RIGHT', clr_r)


def wait_for_key():
    blink('AMBER', 'AMBER')
    Button().wait_for_bump('enter')
    blink('GREEN', 'GREEN')


# ------------------------------VOID------------------------------
def len_to_degrees(length):
    return 180 * length // wheel_radius * 3.14


# ---------MOTORS-------------
def rotate_90_left():
    angle = 90 * rotate_radius // wheel_radius  # 2*pi*R * 1/4 // 2 * pi* r // 90
    motors.on_for_degrees(steering=-100, speed=motor_speed, degrees=angle)


def rotate_90_right():
    angle = 90 * rotate_radius // wheel_radius  # 2*pi*R * 1/4 // 2 * pi* r // 90
    motors.on_for_degrees(steering=100, speed=motor_speed, degrees=angle)


def rotate_180_left():
    angle = 2 * 90 * rotate_radius // wheel_radius  # 2*pi*R * 1/4 // 2 * pi* r // 90
    motors.on_for_degrees(steering=-100, speed=motor_speed, degrees=angle)


def rotate_180_right():
    angle = 2 * 90 * rotate_radius // wheel_radius  # 2*pi*R * 1/4 // 2 * pi* r // 90
    motors.on_for_degrees(steering=100, speed=motor_speed, degrees=angle)


def move_to_cross_center():
    motors.on_for_degrees(steering=0, speed=motor_speed, degrees=len_to_degrees(cross_center_dist))


def line_move(limit=0):
    motor_l_enc = motor_l.position_sp
    motor_r_enc = motor_r.position_sp
    condition = (sens_cross.reflected_light_intensity > middle_val) if limit == 0 else \
        (motor_l.position_sp - motor_l_enc < limit and motor_r.position_sp - motor_r_enc < limit)
    while condition:
        p_val = (sens_line.reflected_light_intensity - middle_val - middle_val) * kp
        speed_l = p_val if p_val < 0 else -p_val
        motors_t.on(speed_l, motor_speed)

    motors_t.stop()


# ------------------------------MAIN------------------------------
def setup():
    blink('GREEN', 'GREEN')


def main():
    setup()

    line_move()
    rotate_90_left()
    line_move(dist_to_cube)
    rotate_180_right()
    line_move()
    rotate_90_left()
    while Button().buttons_pressed != ['backspace']:
        print('OK')
        time.sleep(1)


if __name__ == '__main__':
    while True:
        blink('RED', 'RED')
        Button().wait_for_bump('enter')
        main()