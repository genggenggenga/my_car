# -*- coding:UTF-8 -*-
import RPi.GPIO as GPIO
import time

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

from car_run import *
from infrared_distance import *
from infrared_light import *
from key import *
from led import *
from servo import *
from ultrasonic import *

# 按下开关开始启动，再按一下熄火
on_off = 0
dir_list=[]

def key_pressed_callback(pin):
    global on_off
    if not on_off:
        on_off = 1
    else:
        brake()
        car_run_clean()
        servo_clean()
        led_clean()
        exit(0)


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
        time.sleep(spin_time)
    elif left_sensor == GPIO.HIGH and right_sensor == GPIO.LOW:  # 右边有障碍
        print("右边有障碍")
        left(spin_speed)
        time.sleep(spin_time)
    else:
        print("左右都有障碍")  # 左右都有障碍
        spin_right(spin_speed)
        time.sleep(spin_time)


def runAndLight_according_distance():
    '''
    舵机旋转超声波测距避障,led根据车的状态显示相应的颜色并选择行驶模式
     '''
    # 品红色
    LED_RGB(GPIO.HIGH, GPIO.LOW, GPIO.HIGH)

    back(LOW_SPEED, LOW_SPEED)
    time.sleep(0.18)
    brake()

    # 舵机旋转到0度，即右侧，测距
    servo_appointed_detection(0)
    time.sleep(0.8)
    rightdistance = get_distance()

    # 舵机旋转到180度，即左侧，测距
    servo_appointed_detection(160)
    time.sleep(0.8)
    leftdistance = get_distance()

    # 舵机旋转到90度，即前方，测距
    servo_appointed_detection(75)
    time.sleep(0.8)
    frontdistance = get_distance()

    if leftdistance < DISTANCE_1 and rightdistance < DISTANCE_1 and frontdistance < DISTANCE_1:
        # 亮红色，掉头
        LED_RGB(GPIO.HIGH, GPIO.LOW, GPIO.LOW)
        spin_right(SPIN_MID_SPEED)
        time.sleep(0.6)
    elif leftdistance >= rightdistance:
        # 亮蓝色
        LED_BLUE()
        spin_left(SPIN_MID_SPEED)
        time.sleep(0.15)
    elif leftdistance <= rightdistance:
        # 亮蓝色
        LED_BLUE()
        spin_right(SPIN_MAX_SPEED)
        time.sleep(0.15)


def choose_path():
    while 1:
        dis = get_distance()
        if dis < 20:
            left_sensor_dis = leftSensorValue_distance()
            right_sensor_dis = rightSensorValue_distance()
            if left_sensor_dis == GPIO.LOW and right_sensor_dis == GPIO.LOW:  # 后续补充先获取两侧距离的版本
                while leftSensorValue_distance() == GPIO.LOW:
                    spin_right(SPIN_MID_SPEED)
            elif left_sensor_dis == GPIO.LOW and right_sensor_dis == GPIO.HIGH:
                while leftSensorValue_distance() == GPIO.LOW:
                    spin_right(SPIN_MID_SPEED)
            elif left_sensor_dis == GPIO.HIGH and right_sensor_dis == GPIO.LOW:
                while rightSensorValue_distance() == GPIO.LOW:
                    spin_left(SPIN_MID_SPEED)
        elif dis > 50:
            run(MID_SPEED, MID_SPEED)
        else:
            run(LOW_SPEED, LOW_SPEED)


def choose_dir():
    '''
    dir_list:0  1    2   3    4
             前 左45 左90 右45 右90
    :return:
    '''
    dir_list.clear()

    def get_dis_list(pos,sleep_time):
        servo_appointed_detection(pos)
        time.sleep(sleep_time)
        dir_list.append(get_distance())
    get_dis_list(75,0.1)
    get_dis_list(37,0.4)
    get_dis_list(0,0.4)
    get_dis_list(117,1.2)
    get_dis_list(160,0.4)
    servo_appointed_detection(75)
    time.sleep(0.8)
    d_0=dir_list[2]
    d_45=dir_list[1]
    d_90=dir_list[0]
    d_135=dir_list[3]
    d_180=dir_list[4]

    if d_180 - d_90>30:          #右转90
        spin_right(MID_SPEED,0.3)
    elif d_135 - d_90>30:        #右转45
        spin_right(MID_SPEED,0.15)
    elif d_180 - d_90 < 30:      #右转135
        spin_right(MID_SPEED,0.45)
    elif d_0 - d_90 > 30:        #左转90
        spin_left(MID_SPEED, 0.3)
    elif d_45 - d_90>30:         #左转45
        spin_left(MID_SPEED,0.15)
    elif d_0 - d_90<30:          #左转135
        spin_left(MID_SPEED,0.45)

def choose_dir_print():
    '''
        dir_list:0  1    2   3    4
                 前 左45 左90 右45 右90
        :return:
        '''
    dir_list.clear()

    def get_dis_list(pos, sleep_time):
        servo_appointed_detection(75)
        time.sleep(0.8)
        dir_list.append(get_distance())

    get_dis_list(75, 0.1)
    get_dis_list(37, 0.4)
    get_dis_list(0, 0.4)
    get_dis_list(117, 1.2)
    get_dis_list(160, 0.4)
    servo_appointed_detection(75)
    time.sleep(0.8)
    d_0 = dir_list[2]
    d_45 = dir_list[1]
    d_90 = dir_list[0]
    d_135 = dir_list[3]
    d_180 = dir_list[4]

    if d_180 - d_90 > 30:  # 右转90
        print("右转90")
    elif d_135 - d_90 > 30:  # 右转45
        print("右转45")
    elif d_0 - d_90 > 30:  # 左转90
        print("左转90")
    elif d_45 - d_90 > 30:  # 左转45
        print("左转45")
    elif d_0 - d_90 <30 and d_180 - d_90 < 30:
        spin_right_angle(135)
        dis=get_distance()
        if dis<40:
            spin_right_angle(90)
            dis=get_distance()
            if dis<40:
                spin_left_angle(45)
                print("掉头")
            else:
                print("左转135")
        else:
            print("右转135")
    time.sleep(3)



if __name__ == "__main__":
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    car_run_init()
    infrared_distance_init()
    # infrared_light_init()
    key_init()
    led_init()
    servo_init()
    ultrasonic_init()

    GPIO.add_event_detect(key, GPIO.RISING, key_pressed_callback, bouncetime=15)
    #choose_path()
    choose_dir_print()

    car_run_cleanup()
    servo_cleanup()
    led_cleanup()
