# 导入系统库
import cv2 as cv
import numpy as np
import time
from color_interval import *
# 导入自定义库
from tools import region_of_interest, detect_line, average_lines, display_line

steer_angle =0

def trackbar_callback(value):
    pass

def track_line(cap,color:str):
    '''
    线程函数
    '''
    global steer_angle

    cv.namedWindow("mask", cv.WINDOW_AUTOSIZE)
    cv.namedWindow("edge", cv.WINDOW_AUTOSIZE)
    cv.namedWindow("black", cv.WINDOW_AUTOSIZE)

    cv.createTrackbar("low_threshold", "edge", 0, 1000, trackbar_callback)
    cv.createTrackbar("high_threshold", "edge", 0, 1000, trackbar_callback)
    
    color_lower,color_upper=color_dict[color]
    
    cnt = 0
    while 1:
        ret, frame = cap.read()
        if not ret:
            continue
        height, width, _ = frame.shape
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv, color_lower, color_upper)
        mask = cv.erode(mask, None, iterations=2)
        # mask = cv.dilate(mask, None, iterations=1)
        # mask = cv.GaussianBlur(mask, (3, 3), 0)
        cv.imshow("mask",mask)
        # 边缘检测

        thresh1 =cv.getTrackbarPos("low_threshold","edge")
        thresh2 =cv.getTrackbarPos("high_threshold", "edge")

        edge = cv.Canny(mask, thresh1, thresh2)
        cv.imshow("edge",edge)

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
        show = display_line(left_frame, left_lane, line_color=(255, 0, 0),line_width=20)
        show = display_line(show, right_lane, line_color=(0, 255, 0),line_width=20)

        cv.imshow("black", show)

        # 计算转向角
        x_offset = 0    #一般为几十
        y_offset = 0    #固定为window_height/2=240
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
            # break
            pass

        #print("x:", x_offset)
        #print("y:", y_offset)
        if y_offset==0:
            continue
        steer_angle = x_offset*10 // y_offset
        #print("steer_angle:",steer_angle)
        cnt += 1
        #time.sleep(5)
        #print("speed",30*(1+0.1*abs(steer_angle)))

        key = cv.waitKey(1)
        if key == ord("q"):
            break
        elif key == ord("c"):
            cv.imwrite("cut" + str(cnt) + ".jpg", show)

    cv.destroyAllWindows()

if __name__=="__main__":
    cap=cv.VideoCapture(0)
    while 1:
        track_line(cap,"white")

