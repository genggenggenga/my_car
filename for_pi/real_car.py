import cv2 as cv
import RPi.GPIO as GPIO
import time

from car_tools.key import *
from color_interval import *
from tools import *
from car_tools.car_run import *


red_cnt=0
green_cnt=0
yellow_cnt=0
flag_1s=30
#连续亮灯到达一秒 才算是识别到 但由于算力的问题 需要修改



on_off=0    #按键是否按下
key_pressed_first=0
steer_angle = 0


def key_pressed_callback(pin):
    global on_off,key_pressed_first
    if not on_off:
        on_off = 1
        time.sleep(0.5)
    else:
        brake()
        exit(0)
        
def find_contours(hsv_frame,color:str):
    color_lower,color_upper=color_dict[color]
    mask = cv.inRange(hsv_frame, color_lower, color_upper)
    mask = cv.erode(mask, None, iterations=1)
    #mask = cv.dilate(mask, None, iterations=2)
    #mask = cv.GaussianBlur(mask, (3, 3), 0)
    contours = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[-2]
    if len(contours)==0:
        return 0,0,0,0
    contour = max(contours, key=cv.contourArea)
    (x,y), radius = cv.minEnclosingCircle(contour)
    return x,y,radius,contour

def trackbar_callback(value):
    pass

if __name__ == '__main__':
    key_init()
    car_run_init()
    GPIO.add_event_detect(key, GPIO.RISING, key_pressed_callback, bouncetime=15)
    cap=cv.VideoCapture(0)

    cv.namedWindow("mask", cv.WINDOW_AUTOSIZE)
    cv.namedWindow("edge", cv.WINDOW_AUTOSIZE)
    cv.namedWindow("video", cv.WINDOW_AUTOSIZE)
    cv.namedWindow("traffic_light", cv.WINDOW_AUTOSIZE)

    #canny的阈值
    cv.createTrackbar("low_threshold", "edge", 0, 1000, trackbar_callback)
    cv.createTrackbar("high_threshold", "edge", 0, 1000, trackbar_callback)

    color_lower, color_upper = color_dict["black"]

    cnt = 0   #截图序号


    while 1:
        ret, frame = cap.read()
        if not ret:
            print("video read error!")
            break

        '''
        track line
        '''
        height, width, _ = frame.shape
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv, color_lower, color_upper)
        mask = cv.erode(mask, None, iterations=1)
        # mask = cv.dilate(mask, None, iterations=1)
        # mask = cv.GaussianBlur(mask, (3, 3), 0)
        cv.imshow("mask", mask)

        # 边缘检测
        thresh1 = cv.getTrackbarPos("low_threshold", "edge")
        thresh2 = cv.getTrackbarPos("high_threshold", "edge")

        edge = cv.Canny(mask, thresh1, thresh2)
        cv.imshow("edge", edge)

        # region of interest 获取感兴趣区域的边缘
        black_left_roi = region_of_interest(edge, color="left")
        black_right_roi = region_of_interest(edge, color="right")

        left_frame = frame.copy()
        right_frame = frame.copy()

        # 线段检测
        left_lines = detect_line(black_left_roi)
        right_lines = detect_line(black_right_roi)
        left_lane = average_lines(frame, left_lines, direction='left')
        right_lane = average_lines(frame, right_lines, direction="right")
        show = display_line(left_frame, left_lane, line_color=(255, 0, 0), line_width=20)
        show = display_line(show, right_lane, line_color=(0, 255, 0), line_width=20)
        cv.imshow("video", show)

        # 计算转向角
        x_offset = 0
        y_offset = 0
        if len(left_lane) > 0 and len(right_lane) > 0:  # 检测到2条线
            _, _, left_x2, _ = left_lane[0][0]
            _, _, right_x2, _ = right_lane[0][0]
            mid = int(width / 2)
            x_offset = (left_x2 + right_x2) / 2 - mid
            y_offset = int(height / 2)
        elif len(left_lane) > 0 and len(left_lane[0]) == 1:  # 只检测到黄色行道线
            x1, _, x2, _ = left_lane[0][0]
            x_offset = x2 - x1
            y_offset = int(height / 2)
        elif len(right_lane) > 0 and len(right_lane[0]) == 1:  # 只检测到白色行道线
            x1, _, x2, _ = right_lane[0][0]
            x_offset = x2 - x1
            y_offset = int(height / 2)
        else:  # 一条线都没检测到
            print('检测不到行道线')

        # print("x:", x_offset)
        # print("y:", y_offset)
        if y_offset == 0:
            continue
        steer_angle = x_offset * 10// y_offset
        turn_speed=30*(1+0.1*steer_angle)
        if turn_speed>80:
            turn_speed=80
        '''
        signal light
        '''
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

        red_x, red_y, red_radius, red_contour = find_contours(hsv, "red")
        green_x, green_y, green_radius, green_contour = find_contours(hsv, "green")
        yellow_x, yellow_y, yellow_radius, yellow_contour = find_contours(hsv, "yellow")

        max_radius = max(red_radius, green_radius, yellow_radius)
        if max_radius < 50:
            pass
        if max_radius == red_radius:
            max_x, max_y = red_x, red_y
            cv.circle(frame, (int(max_x), int(max_y)), int(max_radius), (0, 0, 255), 2)
            if on_off:
                red_cnt += 1
            print("red_cnt", red_cnt)

        elif max_radius == green_radius:
            max_x, max_y = green_x, green_y
            cv.circle(frame, (int(max_x), int(max_y)), int(max_radius), (0, 255, 0), 2)
            if on_off:
                green_cnt += 1
            print("green_cnt", green_cnt)

        else:
            max_x, max_y = yellow_x, yellow_y
            cv.circle(frame, (int(max_x), int(max_y)), int(max_radius), (0, 255, 255), 2)
            if on_off:
                yellow_cnt += 1
            print("yellow_cnt", yellow_cnt)

        cv.imshow("traffic_light", frame)

        key = cv.waitKey(1)
        if key == ord("q"):
            break
        elif key == ord("c"):
            cv.imwrite("cut" + str(cnt) + ".jpg", show)
        cnt += 1

        '''
        小车运动
        '''
        if on_off:
            if key_pressed_first==0:
                time.sleep(5)
                key_pressed_first=1

            run(10,10)
            if steer_angle>=0:
                right(turn_speed)
                time.sleep(0.1)
                run()
            else:
                left(turn_speed)
                time.sleep(0.1)

            if red_cnt==flag_1s:
                brake()
                red_cnt=0
                green_cnt=0
                yellow_cnt=0
                print("brake")

            if green_cnt==flag_1s:
                run(10,10)
                red_cnt=0
                green_cnt=0
                yellow_cnt=0
                time.sleep(0.05)
                print("run")

            if yellow_cnt==flag_1s:
                #减速
                red_cnt=0
                green_cnt=0
                yellow_cnt=0
                curr_speed=10
                while curr_speed>=0:
                    curr_speed-=5
                    run(curr_speed,curr_speed)
                    time.sleep(1)
                    print("slow down")
          