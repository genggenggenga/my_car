import cv2 as cv
import numpy as np
import math

import time
from color_interval import *
import threading
import track_line
import signal_light

#from tcp_control.car_run import *

def cnt_lock(lock):
    lock.acquire()
    signal_light.red_cnt = signal_light.green_cnt = signal_light.yellow_cnt = 0
    lock.release()


if __name__ == '__main__':
    cap=cv.VideoCapture(0)

    t1 = threading.Thread(target=track_line.track_line, args=[cap,"black"])
    t2 = threading.Thread(target=signal_light.traffic_light, args=[cap])
    t1.start()
    t2.start()

    while 1:
        #run(LOW_SPEED,LOW_SPEED)
        if track_line.steer_angle>=0:
            #right(LOW_SPEED)
            pass
        else:
            #left(LOW_SPEED)
            pass

        if signal_light.red_cnt==signal_light.flag_1s:
            #brake()
            print("brake")
            cnt_lock(signal_light.threadLock)

        if signal_light.green_cnt==signal_light.flag_1s:
            #run(LOW_SPEED,LOW_SPEED)
            print("run")
            cnt_lock(signal_light.threadLock)
        if signal_light.yellow_cnt==signal_light.flag_1s:
            #减速
            cnt_lock(signal_light.threadLock)
            print("yellow")
            pass