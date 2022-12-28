#-*- coding:UTF-8 -*-
"""
本次舵机转动控制七彩灯控制舵机采用的是系统的pwm库
"""
import RPi.GPIO as GPIO
import time

#开关
on_off=0

#舵机引脚定义
servoPin = 23

global pwm_servo

def servo_init():
    global pwm_servo
    GPIO.setup(servoPin, GPIO.OUT)
    pwm_servo = GPIO.PWM(servoPin, 50)
    pwm_servo.start(0)

    #设置pwm引脚和频率为50hz
def servo_pause():
    GPIO.output(servoPin,GPIO.LOW)


def servo_clean():
    global pwm_servo
    pwm_servo.stop()
    GPIO.cleanup(servoPin)


#舵机旋转到指定角度
def servo_appointed_detection(pos):
    '''
    :param pos: 角度
    160：最左边
    0：最右边
    75：中间
    一般要sleep 0.8秒
    '''
    global pwm_servo
    pwm_servo.ChangeDutyCycle(2.5 + 10 * pos/180)
    time.sleep(0.01)


if __name__=="__main__":
    while 1:
        #for i in range(0,160):
        #    servo_appointed_detection(i)
        #    time.sleep(0.01)
        #for i in range(160,0,-1):
        #    servo_appointed_detection(i)
        #    time.sleep(0.01)
        #time.sleep(1)
        
        servo_appointed_detection(160)
        time.sleep(0.8)
        servo_appointed_detection(75)
        time.sleep(0.8)
        servo_appointed_detection(0)
        time.sleep(0.8)
        
        
        
