#-*- coding:UTF-8 -*-
import RPi.GPIO as GPIO

#速度常量
ZERO_SPEED          =0
LOW_SPEED           =10
MID_SPEED           =30
MAX_SPEED           =100
SPIN_LOW_SPEED      =10
SPIN_MID_SPEED      =40
SPIN_MAX_SPEED      =70

#小车电机引脚定义，A控制左侧，B控制右侧。
AIN1 = 21
AIN2 = 20

BIN1 = 26
BIN2 = 19

PWMA = 16
PWMB = 13

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(PWMA,GPIO.OUT,initial=GPIO.LOW)
GPIO.setup(PWMB,GPIO.OUT,initial=GPIO.LOW)
pwm_PWMA = GPIO.PWM(PWMA, 2000)
pwm_PWMB = GPIO.PWM(PWMB, 2000)
pwm_PWMA.start(0)
pwm_PWMB.start(0)

#电机引脚初始化操作
def car_run_init():
    global pwm_PWMA
    global pwm_PWMB

    #初始化引脚
    GPIO.setup(PWMA,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(AIN2,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(AIN1,GPIO.OUT,initial=GPIO.LOW)

    GPIO.setup(PWMB,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(BIN2,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(BIN1,GPIO.OUT,initial=GPIO.LOW)

    #设置pwm引脚和频率为2000hz
    #pwm_PWMA = GPIO.PWM(PWMA, 2000)
    #pwm_PWMB = GPIO.PWM(PWMB, 2000)
    

    #GPIO.add_event_detect(key,GPIO.RISING,key_pressed_callback,bouncetime=15)


def car_run_clean():
    pwm_PWMA.stop()
    pwm_PWMB.stop()
    cleanList=[AIN1,AIN2,BIN1,BIN2,PWMA,PWMB]
    GPIO.cleanup(cleanList)

#小车前进
def run(leftSpeed,rightSpeed):
    GPIO.output(AIN2, GPIO.HIGH)
    GPIO.output(AIN1, GPIO.LOW)
    GPIO.output(BIN2, GPIO.HIGH)
    GPIO.output(BIN1, GPIO.LOW)
    pwm_PWMA.ChangeDutyCycle(leftSpeed)
    pwm_PWMB.ChangeDutyCycle(rightSpeed)

#小车后退
def back(leftSpeed,rightSpeed):
    GPIO.output(AIN2, GPIO.LOW)
    GPIO.output(AIN1, GPIO.HIGH)
    GPIO.output(BIN2, GPIO.LOW)
    GPIO.output(BIN1, GPIO.HIGH)
    pwm_PWMA.ChangeDutyCycle(leftSpeed)
    pwm_PWMB.ChangeDutyCycle(rightSpeed)

#小车左转
def left(speed):
    GPIO.output(AIN2, GPIO.LOW)
    GPIO.output(AIN1, GPIO.LOW)
    GPIO.output(BIN2, GPIO.HIGH)
    GPIO.output(BIN1, GPIO.LOW)
    pwm_PWMA.ChangeDutyCycle(ZERO_SPEED)
    pwm_PWMB.ChangeDutyCycle(speed)

#小车右转
def right(speed):
    GPIO.output(AIN2, GPIO.HIGH)
    GPIO.output(AIN1, GPIO.LOW)
    GPIO.output(BIN2, GPIO.LOW)
    GPIO.output(BIN1, GPIO.LOW)
    pwm_PWMA.ChangeDutyCycle(speed)
    pwm_PWMB.ChangeDutyCycle(ZERO_SPEED)

#小车原地左转
def spin_left(spinSpeed):
    GPIO.output(AIN2, GPIO.LOW)
    GPIO.output(AIN1, GPIO.HIGH)
    GPIO.output(BIN2, GPIO.HIGH)
    GPIO.output(BIN1, GPIO.LOW)
    pwm_PWMA.ChangeDutyCycle(spinSpeed)
    pwm_PWMB.ChangeDutyCycle(spinSpeed)

#小车原地右转
def spin_right(spinSpeed):
    GPIO.output(AIN2, GPIO.HIGH)
    GPIO.output(AIN1, GPIO.LOW)
    GPIO.output(BIN2, GPIO.LOW)
    GPIO.output(BIN1, GPIO.HIGH)
    pwm_PWMA.ChangeDutyCycle(spinSpeed)
    pwm_PWMB.ChangeDutyCycle(spinSpeed)


#小车停止
def brake():
    GPIO.output(AIN2, GPIO.LOW)
    GPIO.output(AIN1, GPIO.LOW)
    GPIO.output(BIN2, GPIO.LOW)
    GPIO.output(BIN1, GPIO.LOW)
