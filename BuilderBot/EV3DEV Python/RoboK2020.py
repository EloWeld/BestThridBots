#!/usr/bin/env python3
import time

from ev3dev2.motor import LargeMotor, MediumMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D, MoveSteering, MoveTank
from ev3dev2.sensor.lego import ColorSensor
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.led import Leds
from ev3dev2.sound import Sound
from ev3dev2.button import Button

# ------------------------------PINS------------------------------
pin_motors = {'LEFT': OUTPUT_A,
              'RIGHT': OUTPUT_D,
              'LIFT': OUTPUT_B,
              'CLAW': OUTPUT_C}

sens_line = ColorSensor(INPUT_1)
sens_cross = ColorSensor(INPUT_4)
sens_mark = ColorSensor(INPUT_3)

# ------------------------------CONSTANS------------------------------
# Movement
wheel_radius = 55 // 2  # mm
rotate_radius = 150 // 2  # mm
motor_speed = -60
cross_center_dist = 125
mark_center_dist = 60

# Claw
lift_max_angle = 105
lift_speed = 50
claw_speed = 100
claw_max_angle = 1800

# Color
white_val = 60
black_val = 10
middle_val = (white_val + black_val) / 2 - 10  # Not middle yet

# PID
kP = 0.9
kD = 3

# Maps
boxes = []
towers = []
building_stage = 'TAKE'
curr_box_color = 0
complete_towers = 0

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
def sign(x):
    return -1 if x < 0 else (1 if x > 0 else 0)


def dist_to_degrees(dist):
    return 180 * dist // wheel_radius * 3.14  # dist / 2 * pi * r / 360


# ---------MOTORS-------------
class MotorsController:
    def __init__(self):
        self.motor_left = LargeMotor(pin_motors['LEFT'])
        self.motor_right = LargeMotor(pin_motors['RIGHT'])
        self.motor_lift = LargeMotor(pin_motors['LIFT'])
        self.motor_claw = MediumMotor(pin_motors['CLAW'])
        self.motors_s = MoveSteering(pin_motors['LEFT'], pin_motors['RIGHT'])
        self.motors_t = MoveTank(pin_motors['LEFT'], pin_motors['RIGHT'])
        self.lift_state = 'UP'
        self.claw_state = 'OPENED'

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

    def lift_up(self, block=False):
        self.motor_lift.on_for_degrees(speed=-lift_speed, degrees=lift_max_angle, block=block)  # Attention!
        self.lift_state = 'UP'

    def lift_down(self, block=False):
        self.motor_lift.on_for_degrees(speed=lift_speed, degrees=lift_max_angle, block=block)  # Attention!
        self.lift_state = 'DOWN'

    def claw_open(self):
        self.motor_claw.on_for_degrees(speed=motor_speed, degrees=claw_max_angle)  # Attention! Speed signs!
        self.lift_state = 'OPENED'

    def claw_close(self):
        self.motor_claw.on_for_degrees(speed=-motor_speed, degrees=claw_max_angle)  # Attention! Speed signs!
        self.lift_state = 'CLOSED'


def lap_update(mc):
    global building_stage, curr_box_color, complete_towers

    if 0 < sens_mark.color < 7:
        # Build map
        if len(boxes) < 6 and boxes[-1] != sens_mark:
            boxes.append(sens_mark.color)
            print(boxes)
        elif len(towers) < 6 and towers[-1] != sens_mark:
            towers.append(sens_mark.color)
            print(boxes)

        # Stages
        if building_stage == 'TAKE':  # Готовность к взятию коробки(первого)
            curr_box_color = sens_mark.color
            mc.move_to_mark_center()  # Доезжаем чуть вперёд  колёсами, чтобы взять коробку
            mc.lift_down()  # Опускаем лифт, берём коробку, поднимаем её
            mc.claw_close()
            mc.lift_up()
            building_stage = 'DELIVER'  # После взятия коробки просто едем и считываем цвета коробкок
            line_move(mc, lap=True)
            mc.rotate_90_right() # Разворачиваемся и едем до линии с башнями
            line_move(mc, lap=False)
            mc.rotate_90_right()
            building_stage = 'PUT'  # Начинаем ехать по линии со стадией PUT
            line_move(mc, lap=True)
            return True  # End

        if building_stage == 'DELIVER':
            pass

        if building_stage == 'PUT':
            if sens_mark.color == curr_box_color:
                mc.move_to_mark_center()  # Если обнаружили башню цвета схожего с коробкой, то отпускаем коробку
                mc.claw_open()
                complete_towers += 1
                building_stage = 'DELIVER'  # После "строительства" башни просто едем и считываем цилиндры
                line_move(mc, lap=True)
                if complete_towers == 6: # Построили все башни? Едем на финиш
                    mc.rotate_90_right()
                    line_move(mc, lap=False)
                    mc.move_to_finish()
                    return True  # End
                else:  # Не построили? едем на n-ый круг
                    building_stage = 'TAKE'
                    line_move(mc, lap=True)
                    return True  # End?????????

    return False


def line_move(mc, lap=False):
    last_error = 0
    # Cross or distantion
    while sens_cross.reflected_light_intensity > middle_val:  # while not black
        error = sens_line.reflected_light_intensity - middle_val
        # Calculate PID(PD) value
        p_val = error * kP
        d_val = (error - last_error) * kD
        first_result = p_val - d_val
        print('ER: {}; P: {}; D: {}; FV: {}'.format(error, p_val, d_val, first_result))
        # Run motors with speed by PID(P) value
        if first_result > 0:
            result = abs(motor_speed) - first_result
            mc.motors_t.on(left_speed=result * sign(motor_speed), right_speed=motor_speed)
        else:
            result = abs(motor_speed) + first_result
            mc.motors_t.on(left_speed=motor_speed, right_speed=result * sign(motor_speed))
        # Save last_sens_line_val to next while iteration
        last_error = error

        # Lap
        if lap:
            end = lap_update()
            if end:
                break

        # Emergency shutdown
        if Button().buttons_pressed == ['backspace']:
            break

    # Stop motors after cross/distantion_limit
    mc.motors_t.stop()


def move_dist(mc : MotorsController, dist):
    mc.motors_s.on_for_degrees(steering=0, speed=motor_speed, degrees=dist_to_degrees(dist))


# ------------------------------MAIN------------------------------
def setup():
    Debug.blink('GREEN', 'GREEN')


def main():
    setup()
    mc = MotorsController()

    mc.claw_close()
    Button().wait_for_bump(['enter'])
    mc.lift_up()
    Button().wait_for_bump(['enter'])
    mc.lift_down()
    Button().wait_for_bump(['enter'])
    mc.claw_open()
    Button().wait_for_bump(['enter'])
    move_dist(mc, 200)
    time.sleep(2)
    line_move(mc)
    mc.move_to_cross_center()
    Button().wait_for_bump(['enter'])
    mc.rotate_90_left()
    Sound().beep()


if __name__ == '__main__':
    while True:
        Debug.blink('RED', 'RED')
        Button().wait_for_bump('enter')
        main()