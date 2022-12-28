# bgr8转jpeg格式

import enum
import cv2 as cv 


def bgr8_to_jpeg(value, quality=75):
    x=cv.imencode('.jpg', value)[1].tobytes()
    return x
# 摄像头组件显示

import traitlets
import ipywidgets.widgets as widgets
import time
# 线程功能操作库

import threading
import inspect
import ctypes
from IPython.display import display

origin_widget = widgets.Image(format='jpeg', width=320, height=240)
mask_widget = widgets.Image(format='jpeg', width=320, height=240)
result_widget = widgets.Image(format='jpeg', width=320, height=240)
# create a horizontal box container to place the image widget next to eachother
image_container = widgets.HBox([origin_widget, mask_widget, result_widget])
display(image_container)

# 线程相关函数
def _async_raise(tid, exctype):
    """raises the exception, performs cleanup if needed"""
    tid = ctypes.c_long(tid)
    if not inspect.isclass(exctype):
        exctype = type(exctype)

    res = ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, ctypes.py_object(exctype))
    if res == 0:
        raise ValueError("invalid thread id")
    elif res != 1:
        # """if it returns a number greater than one, you're in trouble,
        # and you should call it again with exc=NULL to revert the effect"""
        ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, None)

def stop_thread(thread):
    _async_raise(thread.ident, SystemExit)



# 主进程函数
import numpy as np
import ipywidgets.widgets as widgets

cap = cv.VideoCapture(0)
cap.set(3, 640)
#cap.set(4, 480)
#cap.set(5, 120)  # 设置帧率
cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))

# image.set(cv.CAP_PROP_BRIGHTNESS, 40) #设置亮度 -64 - 64  0.0
# image.set(cv.CAP_PROP_CONTRAST, 50)   #设置对比度 -64 - 64  2.0
# image.set(cv.CAP_PROP_EXPOSURE, 156)  #设置曝光值 1.0 - 5000  156.0

# 默认选择红色的，想识别其他请注释下面红色区间代码，放开后面其他区间代码段
# 红色区间
color_lower = np.array([0, 43, 46])
color_upper = np.array([10, 255, 255])

#绿色区间
color_lower = np.array([35, 43, 46])
color_upper = np.array([77, 255, 255])

#蓝色区间
color_lower=np.array([100, 43, 46])
color_upper = np.array([124, 255, 255])

#黄色区间
color_lower = np.array([26, 43, 46])
color_upper = np.array([34, 255, 255])


#橙色区间
color_lower = np.array([11, 43, 46])
color_upper = np.array([25, 255, 255])

def Color_Recongnize():
    while (1):
        # get a frame and show 获取视频帧并转成HSV格式, 利用cvtColor()将BGR格式转成HSV格式，参数为cv.COLOR_BGR2HSV。
        ret, frame = cap.read()
        #cv.imshow('Capture', frame)
        origin_widget.value = bgr8_to_jpeg(frame)
        # change to hsv model

        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        # get mask 利用inRange()函数和HSV模型中蓝色范围的上下界获取mask，mask中原视频中的蓝色部分会被弄成白色，其他部分黑色。
        mask = cv.inRange(hsv, color_lower, color_upper)
        #cv.imshow('Mask', mask)
        mask_widget.value = bgr8_to_jpeg(mask)
        # detect blue 将mask于原视频帧进行按位与操作，则会把mask中的白色用真实的图像替换：
        res = cv.bitwise_and(frame, frame, mask=mask)
        # cv.imshow('Result', res)
        result_widget.value = bgr8_to_jpeg(res)
        # if cv.waitKey(1) & 0xFF == ord('q'):
        #    break

        time.sleep(0.01)

    cap.release()

    # cv.destroyAllWindows()

#Color_Recongnize()


# 启动进程

thread1 = threading.Thread(target=Color_Recongnize)
thread1.setDaemon(True)
thread1.start()
# 结束进程，只有在结束时才需要执行此段代码

stop_thread(thread1)
