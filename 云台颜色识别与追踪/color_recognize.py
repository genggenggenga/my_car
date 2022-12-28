import cv2 as cv
import numpy as np
cap = cv.VideoCapture(0)
print(cap.isOpened())

str='''请输入你想识别的颜色
red
blue
yellow
green
orange
'''

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

color_dict={"red":[red_lower,red_upper],"green":[green_lower,green_upper],
            "blue":[blue_lower,blue_upper],"yellow":[yellow_lower,yellow_upper]}

def color_recognize():
    choice = input(str)
    while (1):
        ret, frame = cap.read()
        if not ret:
            print("wrong")
            break
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        color_lower,color_upper=color_dict[choice]
        #获取掩膜mask
        mask = cv.inRange(hsv, color_lower, color_upper)
        img = cv.bitwise_and(frame, frame, mask=mask)
        cv.imshow('mask', mask)
        cv.imshow("res",img)
        if cv.waitKey(1000//60) & 0xFF == ord('q'):
            break

    cap.release()
    cv.destroyAllWindows()


color_recognize()
