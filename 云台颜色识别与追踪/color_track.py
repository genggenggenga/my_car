import RPi.GPIO as GPIO
import cv2 as cv
import time
import numpy as np
import PID

# 舵机引脚定义
servoPin1 = 11  # S2
servoPin2 = 9   # S3

color_x = color_y = color_radius = 0
target_valuex = 1400
target_valuey = 1500

xservo_pid = PID.PositionalPID(0.1, 0.2, 0.1)
yservo_pid = PID.PositionalPID(0.1, 0.2, 0.1)

# 红色区间
red_lower = np.array([0, 43, 46])
red_upper = np.array([10, 255, 255])

# #绿色区间
green_lower = np.array([35, 43, 46])
green_upper = np.array([77, 255, 255])

# #蓝色区间
blue_lower=np.array([100, 43, 46])
blue_upper = np.array([124, 255, 255])

# #黄色区间
yellow_lower = np.array([26, 43, 46])
yellow_upper = np.array([34, 255, 255])

# #橙色区间
orange_lower = np.array([11, 43, 46])
orange_upper = np.array([25, 255, 255])

#黑色区间
black_lower = np.array([0,0,0])
black_upper = np.array([180,255,46])


color_dict={"red":[red_lower,red_upper],"green":[green_lower,green_upper],
            "blue":[blue_lower,blue_upper],"yellow":[yellow_lower,yellow_upper],
            "black":[black_lower,black_upper]}

choice_str='''请输入你想识别的颜色
red
blue
yellow
green
black
'''


def init():
    GPIO.setup(servoPin1, GPIO.OUT)
    GPIO.setup(servoPin2, GPIO.OUT)


# 根据舵机脉冲控制范围为500-2500usec内：
def servo_pulse(servo1, servo2):
    '''
    servo1--lower servo
    servo2--higher servo
    '''
    init()
    if servo1 < 500:
        servo1 = 500
    elif servo1 > 2500:
        servo1 = 2500
    if servo2 < 500:
        servo2 = 500
    elif servo2 > 2500:
        servo2 = 2500
    pulsewidth = servo1
    GPIO.output(servoPin1, GPIO.HIGH)
    time.sleep(pulsewidth / 1000000.0)
    GPIO.output(servoPin1, GPIO.LOW)
    time.sleep(20.0 / 1000 - pulsewidth / 1000000.0)
    
    pulsewidth = servo2
    GPIO.output(servoPin2, GPIO.HIGH)
    time.sleep(pulsewidth / 1000000.0)
    GPIO.output(servoPin2, GPIO.LOW)
    time.sleep(20.0 / 1000 - pulsewidth / 1000000.0)
    

def color_track():
    global color_lower, color_upper
    global target_valuex, target_valuey

    times = 0
    choice=input(choice_str)
    color_lower,color_upper=color_dict[choice]

    cv.namedWindow("tracking")
    while True:
        ret, frame = cap.read()
        assert ret,print("摄像头开启失败 in while")

        center_x=frame.shape[1]//2
        center_y=frame.shape[0]//2
        frame = cv.GaussianBlur(frame, (5, 5), 0)
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv, color_lower, color_upper)
        mask = cv.erode(mask, None, iterations=2)
        mask = cv.dilate(mask, None, iterations=2)
        mask = cv.GaussianBlur(mask, (3, 3), 0)
        cnts = cv.findContours(mask.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[-2]

        if len(cnts) > 0:
            cnt = max(cnts, key=cv.contourArea)
            (color_x, color_y), color_radius = cv.minEnclosingCircle(cnt)
            if color_radius > 10:
                times += 1
                cv.circle(frame, (int(color_x), int(color_y)), int(color_radius), (0, 255, 0), 2)

                xservo_pid.SystemOutput = color_x
                xservo_pid.SetStepSignal(center_x)
                xservo_pid.SetInertiaTime(0.01, 0.01)
                target_valuex = int(1400 + xservo_pid.SystemOutput)

                yservo_pid.SystemOutput = color_y
                yservo_pid.SetStepSignal(center_y)
                yservo_pid.SetInertiaTime(0.01, 0.01)
                target_valuey = int(1500 + yservo_pid.SystemOutput)

                # 将云台转动至PID调校位置
                time.sleep(0.008)
                if times == 2:
                    times = 0
                    servo_pulse(target_valuex,target_valuey)

        cv.imshow("tracking",frame)
        if cv.waitKey(33)==ord("q"):
            break

try:
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    cap = cv.VideoCapture(0)

    color_track()
    cap.release()
    cv.destroyAllWindows()

except:
    print("摄像头开启失败 in except")
