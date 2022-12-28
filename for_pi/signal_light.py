import cv2 as cv
import numpy as np
from color_interval import *
import threading
red_cnt=0
green_cnt=0
yellow_cnt=0
flag_1s=1000//30
#连续亮灯到达一秒 才算是识别到 但由于算力的问题 需要修改

threadLock = threading.Lock()



def find_contours(hsv_frame,color:str):
    color_lower,color_upper=color_dict[color]
    mask = cv.inRange(hsv_frame, color_lower, color_upper)
    mask = cv.erode(mask, None, iterations=2)
    mask = cv.dilate(mask, None, iterations=2)
    mask = cv.GaussianBlur(mask, (3, 3), 0)
    contours = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[-2]
    if len(contours)==0:
        return 0,0,0,0
    contour = max(contours, key=cv.contourArea)
    (x,y), radius = cv.minEnclosingCircle(contour)
    return x,y,radius,contour

def traffic_light(cap):
    global red_cnt,green_cnt,yellow_cnt

    cv.namedWindow("traffic_light",cv.WINDOW_AUTOSIZE)
    while 1:
        ret,frame=cap.read()
        if not ret:
            print("in traffic_light thread,video read error!")
            break
        frame = cv.GaussianBlur(frame, (5, 5), 0)
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

        red_x,red_y,red_radius,red_contour=find_contours(hsv,"red")
        green_x,green_y,green_radius,green_contour=find_contours(hsv,"green")
        yellow_x,yellow_y,yellow_radius,yellow_contour=find_contours(hsv,"yellow")

        max_radius=max(red_radius,green_radius,yellow_radius)
        if max_radius<10:
            pass
        if max_radius==red_radius:
            max_x,max_y=red_x,red_y
            cv.circle(frame, (int(max_x),int(max_y)), int(max_radius),(0,0,255),2)

            threadLock.acquire()
            red_cnt += 1
            print("red_cnt", red_cnt)
            threadLock.release()


        elif max_radius==green_radius:
            max_x, max_y = green_x, green_y
            cv.circle(frame, (int(max_x),int(max_y)), int(max_radius), (0, 255, 0), 2)

            threadLock.acquire()
            green_cnt += 1
            print("green_cnt", green_cnt)
            threadLock.release()
        else:
            max_x, max_y = yellow_x, yellow_y
            cv.circle(frame, (int(max_x),int(max_y)), int(max_radius), (0, 255, 255), 2)

            threadLock.acquire()
            yellow_cnt += 1
            print("yellow_cnt", yellow_cnt)
            threadLock.release()

        cv.imshow("traffic_light",frame)
        key=cv.waitKey(1)
        if key==ord("q"):
            break


if __name__=="__main__":
    cap=cv.VideoCapture(0)
    threadLock = threading.Lock()
    while 1:
        ret,frame=cap.read()
        cv.imshow("raw_video",frame)
        if not ret:
            continue
        frame=traffic_light(frame)
        cv.imshow("signal_light",frame)
        if cv.waitKey(1000//30)==ord("q"):
            break

