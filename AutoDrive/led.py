#-*- coding:UTF-8 -*-
import RPi.GPIO as GPIO

#RGB三色灯引脚定义
LED_R = 22
LED_G = 27
LED_B = 24

def led_init():
    GPIO.setup(LED_R, GPIO.OUT)
    GPIO.setup(LED_G, GPIO.OUT)
    GPIO.setup(LED_B, GPIO.OUT)
    
def led_clean():
    led_list=[LED_R,LED_G,LED_B]
    GPIO.cleanup(led_list)

def LED_RGB(R,G,B):
    GPIO.output(LED_R,R)
    GPIO.output(LED_G,G)
    GPIO.output(LED_B,B)

def LED_RED():
    GPIO.output(LED_R,GPIO.HIGH)
    GPIO.output(LED_G,GPIO.LOW)
    GPIO.output(LED_B,GPIO.LOW)

def LED_GREEN():
    GPIO.output(LED_R,GPIO.LOW)
    GPIO.output(LED_G,GPIO.HIGH)
    GPIO.output(LED_B,GPIO.LOW)

def LED_BLUE():
    GPIO.output(LED_R,GPIO.LOW)
    GPIO.output(LED_G,GPIO.LOW)
    GPIO.output(LED_B,GPIO.HIGH)

def led_pause():
    LED_RGB(GPIO.LOW,GPIO.LOW,GPIO.LOW)