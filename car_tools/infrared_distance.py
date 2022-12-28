"""
遇到障碍物,红外避障模块的指示灯亮,端口电平为LOW
未遇到障碍物,红外避障模块的指示灯灭,端口电平为HIGH
"""

#-*- coding:UTF-8 -*-
import RPi.GPIO as GPIO

#红外避障引脚定义
avoidSensorLeft = 12
avoidSensorRight = 17

def infrared_distance_init():
    GPIO.setup(avoidSensorLeft,GPIO.IN)
    GPIO.setup(avoidSensorRight,GPIO.IN)


def leftSensorValue_distance():
    return GPIO.input(avoidSensorLeft)

def rightSensorValue_distance():
    return GPIO.input(avoidSensorRight)

if __name__=="__main__":
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(GPIO.LOW)
    infrared_distance_init()
    while 1:
        left_sensor = leftSensorValue()
        right_sensor=rightSensorValue()
        if left_sensor==GPIO.HIGH and right_sensor==GPIO.HIGH:
            print("左右都没障碍")                             #左右都没有
        elif left_sensor==GPIO.LOW and right_sensor==GPIO.HIGH:     #左边有障碍
            print("左边有障碍")
        elif left_sensor==GPIO.HIGH and right_sensor==GPIO.LOW:     #右边有障碍
            print("右边有障碍")
        else:
            print("左右都有障碍")                             #左右都有障碍

        