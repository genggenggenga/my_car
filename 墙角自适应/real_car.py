# 导入系统库
import cv2 as cv
import numpy as np
import math
#import tcp_control.car_run as car_run
from car_run import *
import time
import RPi.GPIO as GPIO

# 导入自定义库
from tools import region_of_interest, detect_line, average_lines, display_line

cv.namedWindow("black",cv.WINDOW_AUTOSIZE)
#cv.resizeWindow("black",(400,400))

# 白色区域检测
white_lower = np.array([0, 0, 200])
white_upper = np.array([180, 30, 255])

#黑色区间
black_lower = np.array([0, 0, 0])
black_upper = np.array([180 ,255 ,46])

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

def main():
    cap=cv.VideoCapture(0)
    cnt=0
    while 1:
        ret,frame=cap.read()
        if not ret:
            continue
        height, width, _ = frame.shape
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

        black_mask = cv.inRange(hsv, black_lower, black_upper)

        #边缘检测
        black_edge = cv.Canny(black_mask, 100, 200)
        #cv.imshow("yellow_edge",yellow_edge)

        #region of interest 获取感兴趣区域的轮廓
        black_left_roi = region_of_interest(black_edge ,color="black_left")
        black_right_roi = region_of_interest(black_edge, color="black_right")

        black_left_frame = frame.copy()
        black_right_frame = frame.copy()

        #线段检测
        black_left_lines = detect_line(black_left_roi)
        black_right_lines = detect_line(black_right_roi)
        black_left_lane = average_lines(frame, black_left_lines, direction='right')
        black_right_lane = average_lines(frame,black_right_lines, direction="left")
        black_show = display_line(black_left_frame, black_left_lane, line_color=(255, 0, 0))
        black_show = display_line(black_show, black_right_lane, line_color=(255, 0, 0))

        cv.imshow("black",black_show)

        key=cv.waitKey(1000//30)
        if key==ord("q"):
            break
        elif key==ord("c"):
            cv.imwrite("cut"+str(cnt)+".jpg",black_show)
        # 计算转向角
        x_offset = 0
        y_offset = 0
        if len(black_left_lane) > 0 and len(black_right_lane) > 0:  # 检测到2条线
            _, _, left_x2, _ = black_left_lane[0][0]
            _, _, right_x2, _ = black_right_lane[0][0]
            mid = int(width / 2)
            x_offset = (left_x2 + right_x2) / 2 - mid
            y_offset = int(height / 2)
        elif len(black_left_lane) > 0 and len(black_left_lane[0]) == 1:  # 只检测到黄色行道线
            x1, _, x2, _ = black_left_lane[0][0]
            x_offset = x2 - x1
            y_offset = int(height / 2)
        elif len(black_right_lane) > 0 and len(black_right_lane[0]) == 1:  # 只检测到白色行道线
            x1, _, x2, _ = black_right_lane[0][0]
            x_offset = x2 - x1
            y_offset = int(height / 2)
        else:  # 一条线都没检测到
            print('检测不到行道线，退出程序')
            #break

        print("x:",x_offset)
        print("y:",y_offset)
        tanx=abs(x_offset//y_offset) if y_offset!=0 else 0
        if x_offset >=0:
            right(10*tanx)
            time.sleep(0.1)
        else:
            left(10)
            time.sleep(0.1)

        '''
        angle_to_mid_radian = math.atan(x_offset / y_offset)
        angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)
        steering_angle = angle_to_mid_deg / 45.0
        print("angle:",steering_angle)
        action = np.array([steering_angle, 0.3-abs(steering_angle)/5])  # 油门值恒定
        
        obv, reward, done, info = env.step(action)
        frame = cv.cvtColor(obv, cv.COLOR_RGB2BGR)
        '''

        cnt+=1
    # 运行完以后重置当前场景

    cv.destroyAllWindows()

if __name__ == '__main__':
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    car_run_init()
    main()