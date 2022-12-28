#-*- coding:UTF-8 -*-
import RPi.GPIO as GPIO
import time

DISTANCE_1= 30
DISTANCE_2= 50


#超声波引脚定义
echoPin = 0
trigPin = 1


def ultrasonic_init():
    GPIO.setup(echoPin,GPIO.IN)
    GPIO.setup(trigPin,GPIO.OUT)

#超声波函数
def get_distance():
    GPIO.output(trigPin,GPIO.HIGH)
    time.sleep(0.000015)
    GPIO.output(trigPin,GPIO.LOW)
    while not GPIO.input(echoPin):
        #print("等待接受返回信号")
        pass
    t1 = time.time()
    while GPIO.input(echoPin):
        #print("正在接受返回信号")
        pass
    t2 = time.time()
    distance=((t2 - t1)* 340 / 2) * 100
    print ("distance is {}".format(distance))
    time.sleep(0.01)
    #print(distance)
    return distance