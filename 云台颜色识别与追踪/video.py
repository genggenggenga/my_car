import cv2 as cv

cv.namedWindow("window")
vc = cv.VideoCapture(0)  # VideoCapture是个类，vc是对象
print(vc.isOpened())
while vc.isOpened():
    ret, frame = vc.read()
    if not ret:
        break

    cv.imshow("window", frame)
    # 如果视频帧率为30，可设置为 1000 // 30,要取整数
    if cv.waitKey(10) == ord("q"):
        break
vc.release()
cv.destroyAllWindows()