# coding=utf-8
# @Time:2022/11/6 23:38
import numpy as np
import cv2
font = cv2.FONT_HERSHEY_SIMPLEX
lower_green = np.array([35, 110, 106]) # 绿色范围低阈值
upper_green = np.array([77, 255, 255]) # 绿色范围高阈值
lower_red = np.array([0, 127, 128]) # 红色范围低阈值
upper_red = np.array([10, 255, 255]) # 红色范围高阈值
lower_yellow = np.array([26,43, 46]) # 黄色范围低阈值
upper_yellow = np.array([34, 255, 255]) # 黄色范围高阈值
lower_blue = np.array([100,43,46]) # 蓝色范围低阈值
upper_blue = np.array([124, 255, 255]) # 蓝色范围高阈值
cap = cv2.VideoCapture(0) # 打开视频文件
cap = cv2.VideoCapture(0)#打开USB摄像头

if (cap.isOpened()): # 视频打开成功
    flag = 1
else:
    flag = 0
num = 0
if (flag):
    while (True):
        ret, frame = cap.read() # 读取一帧

        if ret == False:  # 读取帧失败
            break
        hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_green = cv2.inRange(hsv_img, lower_green, upper_green)  # 根据颜色范围删选
        mask_red = cv2.inRange(hsv_img, lower_red, upper_red)
        mask_yellow = cv2.inRange(hsv_img, lower_yellow, upper_yellow)
        mask_blue = cv2.inRange(hsv_img, lower_blue, upper_blue)


        mask_green = cv2.medianBlur(mask_green, 7)  # 中值滤波
        mask_red = cv2.medianBlur(mask_red, 7)  # 中值滤波
        mask_yellow = cv2.medianBlur(mask_yellow, 7)  # 中值滤波
        mask_blue = cv2.medianBlur(mask_blue, 7)  # 中值滤波
        mask = cv2.bitwise_or(mask_green, mask_red, mask_yellow, mask_blue)
        mask_green, contours, hierarchy = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        mask_red, contours2, hierarchy2 = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        mask_yellow, contours3, hierarchy3 = cv2.findContours(mask_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        mask_blue, contours5, hierarchy5 = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        for cnt in contours:
            (x, y, w, h) = cv2.boundingRect(cnt)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
            cv2.putText(frame, "Green", (x, y - 5), font, 0.7, (0, 255, 0), 2)

        for cnt2 in contours2:
            (x2, y2, w2, h2) = cv2.boundingRect(cnt2)
            cv2.rectangle(frame, (x2, y2), (x2 + w2, y2 + h2), (0, 255, 255), 2)
            cv2.putText(frame, "Red", (x2, y2 - 5), font, 0.7, (0, 0, 255), 2)

        for cnt in contours:
            (x3, y3, w3, h3) = cv2.boundingRect(cnt3)
            cv2.rectangle(frame, (x3, y3), (x3 + w3, y3 + h3), (0, 255, 255), 2)
            cv2.putText(frame, "yellow", (x3, y3 - 5), font, 0.7, (0, 255, 0), 2)

        for cnt in contours:
            (x4, y4, w4, h4) = cv2.boundingRect(cnt4)
            cv2.rectangle(frame, (x4, y4), (x4 + w4, y4 + h4), (0, 255, 255), 2)
            cv2.putText(frame, "blue", (x4, y4 - 5), font, 0.7, (0, 255, 0), 2)
        num = num + 1
        cv2.imshow("dection", frame)
        cv2.imwrite("imgs/%d.jpg"%num, frame)
        if cv2.waitKey(20) & 0xFF == 27:
            break

cv2.waitKey(0)
cv2.destroyAllWindows()
