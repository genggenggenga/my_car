import RPi.GPIO as GPIO
import cv2 as cv
import time


# 舵机引脚定义
servoPin1 = 11  # x轴
servoPin2 = 9   # y轴

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

def servo_xy_init():
    GPIO.setup(servoPin1, GPIO.OUT)
    GPIO.setup(servoPin2, GPIO.OUT)

def servo_x(angle):
    pulse_width=500+(2000//180)*angle
    GPIO.output(servoPin1, GPIO.HIGH)
    time.sleep(pulse_width / 1000000.0)
    GPIO.output(servoPin1, GPIO.LOW)
    time.sleep(20.0 / 1000 - pulse_width / 1000000.0)

def servo_y(angle):
    pulse_width = 500 + (2000 // 180) * angle
    GPIO.output(servoPin2, GPIO.HIGH)
    time.sleep(pulse_width / 1000000.0)
    GPIO.output(servoPin2, GPIO.LOW)
    time.sleep(20.0 / 1000 - pulse_width / 1000000.0)
    #设置pwm引脚和频率为50hz

def servo_x_pause():
    GPIO.output(servoPin1,GPIO.LOW)

def servo_y_pause():
    GPIO.output(servoPin2,GPIO.LOW)

def servo_clean():
    GPIO.cleanup(servoPin1)
    GPIO.cleanup(servoPin2)
