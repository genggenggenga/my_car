import cv2 as cv
import numpy as np


def callback(x):
    pass


cv.namedWindow("Tracking")
cv.namedWindow("frame")

# 参数：1 Lower/Upper HSV 3 startValue 4 endValue
cv.createTrackbar("LH", "Tracking", 0, 255, callback)
cv.createTrackbar("LS", "Tracking", 0, 255, callback)
cv.createTrackbar("LV", "Tracking", 0, 255, callback)
cv.createTrackbar("UH", "Tracking", 0, 255, callback)
cv.createTrackbar("US", "Tracking", 0, 255, callback)
cv.createTrackbar("UV", "Tracking", 0, 255, callback)

cap = cv.VideoCapture(0)
while True:
    ret, frame = cap.read()
    # frame = cv.imread(r"color.jpg")
    # frame = cv.resize(frame,(480,480))
    if not ret:
        print("VideoCapture error")
        break
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    l_h = cv.getTrackbarPos("LH", "Tracking")
    l_s = cv.getTrackbarPos("LS", "Tracking")
    l_v = cv.getTrackbarPos("LV", "Tracking")
    u_h = cv.getTrackbarPos("UH", "Tracking")
    u_s = cv.getTrackbarPos("US", "Tracking")
    u_v = cv.getTrackbarPos("UV", "Tracking")

    l_g = np.array([l_h, l_s, l_v])
    u_g = np.array([u_h, u_s, u_v])

    mask = cv.inRange(hsv, l_g, u_g)

    res = cv.bitwise_and(frame, frame, mask=mask)

    cv.imshow("frame", frame)
    cv.imshow("mask", mask)
    cv.imshow("res", res)
    key = cv.waitKey(1)
    if key == ord("q"):
        break

cv.destroyAllWindows()
