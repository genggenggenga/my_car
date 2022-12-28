# -*- coding:UTF-8 -*-
import RPi.GPIO as GPIO
from time import *

from car_run import *
from infrared_distance import *
#from infrared_light import *
from key import *
from led import *
from servo import *
from ultrasonic import *


def run_sides_space(run_speed, spin_time=0.2, spin_speed=SPIN_MID_SPEED):
    '''
    遇到障碍物,红外避障模块的指示灯亮,端口电平为LOW
    未遇到障碍物,红外避障模块的指示灯灭,端口电平为HIGH
    '''
    left_sensor = leftSensorValue_distance()
    right_sensor = rightSensorValue_distance()
    print("************", left_sensor)
    if left_sensor == GPIO.HIGH and right_sensor == GPIO.HIGH:
        print("左右都没障碍")  # 左右都没有
        run(run_speed, run_speed)
    elif left_sensor == GPIO.LOW and right_sensor == GPIO.HIGH:  # 左边有障碍
        print("左边有障碍")
        right(spin_speed)
        sleep(spin_time)
    elif left_sensor == GPIO.HIGH and right_sensor == GPIO.LOW:  # 右边有障碍
        print("右边有障碍")
        left(spin_speed)
        sleep(spin_time)
    else:
        print("左右都有障碍")  # 左右都有障碍
        spin_right(spin_speed)
        sleep(spin_time)


def runAndLight_according_distance():
    '''
    舵机旋转超声波测距避障,led根据车的状态显示相应的颜色并选择行驶模式
     '''
    # 品红色
    LED_RGB(GPIO.HIGH, GPIO.LOW, GPIO.HIGH)

    back(LOW_SPEED, LOW_SPEED)
    sleep(0.18)
    brake()

    # 舵机旋转到0度，即右侧，测距
    servo_appointed_detection(0)
    sleep(0.8)
    rightdistance = get_distance()

    # 舵机旋转到180度，即左侧，测距
    servo_appointed_detection(160)
    sleep(0.8)
    leftdistance = get_distance()

    # 舵机旋转到90度，即前方，测距
    servo_appointed_detection(75)
    sleep(0.8)
    frontdistance = get_distance()

    if leftdistance < DISTANCE_1 and rightdistance < DISTANCE_1 and frontdistance < DISTANCE_1:
        # 亮红色，掉头
        LED_RGB(GPIO.HIGH, GPIO.LOW, GPIO.LOW)
        spin_right(SPIN_MID_SPEED)
        sleep(0.6)
    elif leftdistance >= rightdistance:
        # 亮蓝色
        LED_BLUE()
        spin_left(SPIN_MID_SPEED)
        sleep(0.15)
    elif leftdistance <= rightdistance:
        # 亮蓝色
        LED_BLUE()
        spin_right(SPIN_MAX_SPEED)
        sleep(0.15)


def car_auto_drive():
    on_off = 0
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    #car_run_init()
    infrared_distance_init()
    #infrared_light_init()
    key_init()
    led_init()
    servo_init()
    ultrasonic_init()
    
    
    # 按下开关开始启动，再按一下熄火
    def key_pressed_callback(pin):
        nonlocal on_off
        if not on_off:
            on_off = 1
            sleep(0.5)
        else:
            brake()
            #car_run_clean()
            servo_clean()
            #led_clean()
            exit(0)
            

    GPIO.add_event_detect(key, GPIO.RISING, key_pressed_callback, bouncetime=15)
    sleep(2)
    print("Press key2 to poweron!")
    while 1:
        if not on_off:
            brake()
            led_pause()
            servo_pause()
            while not on_off:
                pass

        else:
            distance = get_distance()
            if distance > DISTANCE_2:
                print("distance>50")
                run_sides_space(MID_SPEED)
                # run(MID_SPEED,MID_SPEED)
                LED_GREEN()
            elif distance > DISTANCE_1 and distance <= DISTANCE_2:
                print("30<distance<=50")
                LED_RGB(1, 0, 1)
                run_sides_space(LOW_SPEED)

            else:
                print("distance<30")
                runAndLight_according_distance()

    car_run_cleanup()
    servo_cleanup()
    led_cleanup()
