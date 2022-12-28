#-*- coding:UTF-8 -*-
import RPi.GPIO as GPIO

#小车按键定义
key = 8


def key_init():
    GPIO.setup(key,GPIO.IN)


