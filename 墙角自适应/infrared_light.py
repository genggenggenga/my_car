"""
遇到光线,寻光模块的指示灯灭,端口电平为HIGH
未遇光线,寻光模块的指示灯亮,端口电平为LOW
"""
#-*- coding:UTF-8 -*-
import RPi.GPIO as GPIO

#光敏电阻引脚定义
ldrSensorLeft = 7
ldrSensorRight = 6

def infrared_light_init():
    GPIO.setup(ldrSensorLeft,GPIO.IN)
    GPIO.setup(ldrSensorRight,GPIO.IN)



def leftSensorValue_light():
    return GPIO.input(ldrSensorRight)

def rightSensorValue_light():
    return GPIO.input(ldrSensorRight)