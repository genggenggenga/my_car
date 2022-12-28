# OpenCV

[toc]

## 鼠标回调函数

OpenCV允许我们对窗口上的鼠标动作做出响应。

- **setMouseCallback(winname, callback, userdata) **

  winname 是窗口的名字, callback是回调函数, userdata是给回调函数的参数.

- **callback(event,x, y, flags, userdata)**

  回调函数必须包含这5个参数. event是事件，x、y是点鼠标的坐标点, flags主要用于组合键, userdata就是上面的setMouseCallback的userdata）

- 鼠标事件：

  ```
  EVENT_MOUSEMOVE 	 0 	   鼠标移动
  EVENT_LBUTTONDOWN	 1	   按下鼠标左键
  EVENT_RBUTTONDOWN	 2	   按下鼠标右键
  EVENT_MBUTTONDOWN    3     按下鼠标中键
  EVENT_LBUTTONUP 	 4	   左键释放
  EVENT_RBUTTONUP 	 5	   右键释放
  EVENT_MBUTTONUP 	 6	   中键释放
  EVENT_LBUTTONDBLCLK	 7	   左键双击
  EVENT_RBUTTONDBLCLK  8	   右键双击
  EVENT_MBUTTONDBLCLK  9	   中键双击
  EVENT_MOUSEWHEEL 	 10	   鼠标滚轮上下滚动
  EVENT_MOUSEHWHEEL 	 11    鼠标左右滚动
  ```

  代码如下：

  ```python
  #鼠标事件监控
  def mouse_callback(event,x,y,flags,userdata):
      print(event,x,y,flags,userdata)
      if event==cv.EVENT_LBUTTONDOWN:
          cv.destroyAllWindows()
          exit(0)
  img=cv_imread("D:\桌面\cat.jpg")
  #要先创建窗口
  cv.namedWindow("mouse",cv.WINDOW_AUTOSIZE)
  
  cv.setMouseCallback("mouse",mouse_callback,"data")
  cv_imshow("mouse",img)
  ```

  



## TrackBar

- **createTrackbar(trackbarname, winname, value, count, onChange)**

  创建TrackBar控件, value为trackbar的默认值, count为bar的最大值,最小为0,onChange为回调函数

- **getTrackbarPos(trackbarname, winname)**

  获取TrackBar当前值

  代码如下：

  ```python
  #TrackBar
  cv.namedWindow("TrackBar",cv.WINDOW_AUTOSIZE)
  track_img=np.zeros((480,480,3),np.uint8)
  
  def trackbar_callback(value):
      print(value)
      
  cv.createTrackbar("R","TrackBar",0,255,trackbar_callback)
  cv.createTrackbar("G","TrackBar",0,255,trackbar_callback)
  cv.createTrackbar("B","TrackBar",0,255,trackbar_callback)
  
  while 1:
      r=cv.getTrackbarPos("R","TrackBar")
      g=cv.getTrackbarPos("G","TrackBar")
      b=cv.getTrackbarPos("B","TrackBar")
  
      track_img[:]=[b,g,r]
      cv.imshow("TrackBar",track_img)
      if cv.waitKey(1)==ord("q"):
          break
  cv.destroyAllWindows()
  ```

## 图像与视频处理与播放

### 图像

```python
import cv2 as cv
import numpy as np

#读取图像，解决imread不能读取中文路径路径的问题
def cv_imread(file_path):
    #imdedcode读取的是RGB图像
    cv_img = cv.imdecode(np.fromfile(file_path,dtype=np.uint8),-1)
    return cv_img

img=cv_imread("D:\桌面\我的小车\OpenCV\屏幕截图 2022-09-15 095726.jpg")
cv.namedWindow("blue",cv.WINDOW_NORMAL)
#cv.resizeWindow("blue",472,676)
cv.imshow("blue",img)
cv.waitKey(0)
cv.destroyAllWindows()
```



### 视频

```python
cv.namedWindow("window",cv.WINDOW_NORMAL)
window=cv.resizeWindow("window",(480,720))
vc=cv.VideoCapture("D:\qq聊天记录\MobileFile\VID_20220915_104559.mp4")		#VideoCapture是个类，vc是对象

while vc.isOpened():
    ret,frame=vc.read()
    if not ret:
        break
        
    cv.imshow("window",frame)
    #如果视频帧率为30，可设置为 1000 // 30,要取整数
    if cv.waitKey(10)==ord("q"):
        break
vc.release()
cv.destroyAllWindows()
```



### 图像放大与缩小

resize(**src, dsize**, dst, fx, fy, interpolation)

- src：要放缩的图片

- dsize：变换之后的图片大小,元组和列表表示均可

- dst:可选参数,缩放之后的输出图片

- fx , fy： x轴和y轴的缩放比,即宽度和高度的缩放比

- interpolation:插值算法,主要有以下几种:
  - INTER_NEAREST：邻近插值,速度快,效果差.
  - INTER_LINEAR：双线性插值,使用原图中的4个点进行插值.默认
  - INTER_CUBIC：三次插值,原图中的16个点
  - INTER_AREA：区域插值,效果最好,计算时间最长

```python
new_img=cv.resize(img,(480,480))
new_img=cv.resize(img,dsize=None,fx=0.5,fy=0.5)
```



### 图像叠加

```python
#图像叠加
cat=cv_imread("D:\桌面\我的小车\cat.jpg")
dog=cv_imread("D:\桌面\我的小车\dog.jpg")
dog=cv.resize(dog,(cat.shape[1],cat.shape[0]))
new_img=cv.addWeighted(cat,0.5,dog,0.5,0)
cv_imshow("catAnddog",new_img)
```



### 图像加减乘除

```python
cv.add(img1,img2)
cv.subtract(img1,img2)
cv.multiply(img1,img2)
cv.divide(img1,img2)

#若大于255则为255，小于0则为0；
#但对于直接加减常数的情况，%256
```



### 图像位运算

```python
#位运算
img_not=cv.bitwise_not(img)         #255-img
img_and=cv.bitwise_and(img1,img2)
img_or=cv.bitwise_or(img1,img2)
img_xor=cv.bitwise_xor(img1,img2)
```





### 颜色空间转换

```python
cat=cv_imread("D:\桌面\cat.jpg")
cv.namedWindow("con_window")

def callback(value):
    print("当前颜色空间为",cv.getTrackbarPos("traceBar","con_window"),sep="")

color_space=[cv.COLOR_BGR2BGRA,cv.COLOR_BGR2RGB,cv.COLOR_BGR2GRAY,cv.COLOR_BGR2HSV,cv.COLOR_BGR2YUV]
cv.createTrackbar("traceBar","con_window",0,3,callback)
while 1:
    pos=cv.getTrackbarPos("traceBar","con_window")
    cvt_img=cv.cvtColor(cat,color_space[pos] )
    cv.imshow("con_window",cvt_img)
    if cv.waitKey(10)==ord("q"):
        break
cv.destroyAllWindows()
```



### 图形绘制

利用OpenCV提供的绘制图形API可以轻松在图像上绘制各种图形,比如直线,矩形,圆,椭圆等图形.

- line(**img, pt1 , pt2, color, thickness, lineType, shift**)

  - img：在哪个图像上画线
  - pt1, pt2：开始点,结束点.指定线的开始与结束位置o color:颜色

  - thickness：线宽

  - lineType：线型，线型为1,4,8,16，默认为8 

  - shift：坐标缩放比例

- rectangle(**img, pt1 , pt2, color, thickness, lineType, shift**)

- circle(**img, center, radius, color**, thickness, lineType, shift)中括号内参数表示可选参数.画圆。 

- ellipse(**img,中心点,(a,b),角度,从哪个角度开始,从哪个角度结束**)  椭圆

- polylines(**img, pts, isClosed, color**, thickness, lineType, shift)     画多边形

  - pts是点集的集合，要加[]

- fillPoly(**img, pts, isClosed, color**, thickness, lineType, shift)         填充多边形

- putText(**img, text, org, fontFace, fontScale, color**, thickness, lineType, bottomLeftOrigin)      绘制文本

  - text要绘制的文本
  - org 文本在图片中的左下角坐标
  - fontFace字体类型即字体，如 cv.FONT_ITALIC   
  - fontScale字体大小，如1,2,3

  

```python
cv.namedWindow("draw")
board=np.zeros((480,480,3))
cv.line(board,(0,0),(480,480),(0,0,255),lineType=1)
cv.line(board,(480,0),(0,480),(0,0,255),lineType=1)
cv.rectangle(board,(120,120),(360,360),(255,0,0),lineType=1)
cv.circle(board,(240,240),120,(120,55,10),lineType=1)
cv.ellipse(board,(240,240),(120,60),90,0,360,(0,255,0),lineType=1)
cv.imshow("draw",board)
cv.waitKey(0)
cv.destroyAllWindows()
```







## 图像的基本变换

### 图像翻转

- flip(src,flipCode)
  - flipCode=0  上下翻转
  - flipCode>0  左右翻转
  - flipCode<0  上下+左右



### 图像旋转

- rotate(src,rotateCode)
  - ROTATE_90_CLOCKWISE 				   顺时针90°
  - ROTATE_180                                        180°
  - ROTATE_90_COUNTERCLOCKWISE  逆时针90°



### 仿射变换

#### 平移

反射变换是图像旋转、缩放和平移的总称。具体做法是通过一个矩阵和原图片坐标进行计算，得到新的坐标，完成变换。关键就是变换矩阵。

- warpAffine(**src,M,dsize**,dst,flags,borderMode,borderValue)

  - M：变换矩阵，至少是float32
  - dsize：输出图片大小
  - flags：插值算法
    - INTER_NEAREST：邻近插值,速度快,效果差.
    - INTER_LINEAR：双线性插值,使用原图中的4个点进行插值.默认
    - INTER_CUBIC：三次插值,原图中的16个点
    - INTER_AREA：区域插值,效果最好,计算时间最长
  - borderMode：边界外推标志
  - borderValue：填充边界值

- 平移矩阵

  若平移后坐标为(x+tx,y+ty)，矩阵表示法为

![image-20220923195425794](https://gitee.com/genggenggenga/Picture/raw/master/images/image-20220923195425794.png)

```python
#仿射变换
cat=cv_imread("D:\桌面\我的小车\cat.jpg")
cv.namedWindow("window")

height,width,channel=cat.shape
M=np.float32([[1,0,200],[0,1,100]])
new_cat=cv.warpAffine(cat,M,dsize=(width,height))
cv.imshow("window",np.hstack((cat,new_cat)))		#cv.hstack()要传入元组
cv.waitKey(0)
cv.destroyAllWindows()
```



#### 旋转

仿射变换的难点就在于计算变换矩阵，OpenCV提供了计算变换矩阵的API

- getRotationMatrix2D(center,angle,scale)
  - center：	旋转中心
  - angle：      旋转角度，整数逆时针旋转，负数顺时针
  - scale：       缩放比例，例如0.5,1,2

```python
M=cv.getRotationMatrix2D((width/2,height/2),30,1)
new_cat=cv.warpAffine(cat,M,dsize=(width,height))
cv.imshow("window",np.hstack((cat,new_cat)))
cv.waitKey(0)
cv.destroyAllWindows()
```



- getAffineTransform(src[],dst[])	

  - 输入三点，根据三点的变换进行旋转
  - 两个矩阵至少是float32


```py
#三点旋转
src=np.float32([[20,20],[100,100],[200,150]])
dst=np.float32([[10,30],[60,70],[100,100]])
M=cv.getAffineTransform(src,dst)
new_cat=cv.warpAffine(cat,M,(width,height))
```





### 透视变换

- warpPerspective(**src,M,dsize**,dst,flags,borderMode,borderValue)
- getPerspectiveTransform(src,dst)
  - 获取透视变换的变换矩阵，需要四个点，即照片的四个角

```python
#当图片太大时，需要重新定义窗口大小
img=cv_imread("D:\桌面\我的小车\透视变换.jpg")
src=np.float32([[1000,2500],[0,4095],[2000,2500],[3071,4095]])
dst=np.float32([[0,0],[0,2047],[3071,0],[3071,2047]])
M=cv.getPerspectiveTransform(src,dst)
new_img=cv.warpPerspective(img,M,(3072,2048))
print(M)          #M是3*3的矩阵
cv.namedWindow("window",cv.WINDOW_NORMAL)
cv.resizeWindow("window",720,480)
cv_imshow("window",new_img)
```



## 滤波器

### 卷积

- filter2D(**src,ddepth,kernel**,dst,anchor,delta,borderType)

  - ddepth：卷积之后图片的位深,即卷积之后图片的数据类型,一般设为-1,表示和原图类型一致
  - kernel：卷积核大小,用元组或者ndarray表示,要求数据类型必须是float型
  - anchor：锚点,即卷积核的中心点,是可选参数,默认是(-1,-1)
  - delta：可选参数,表示卷积之后额外加的一个值,相当于线性方程中的偏差,默认为0
  - borderType：边界类型.一般不设

  ```python
  kernel1=np.ones((5,5),np.float32)/25         				#平滑
  dst1=cv.filter2D(cat,-1,kernel1)
  
  kernel2=np.array([[-1,-1,-1],[-1,9,-1],[-1,-1,-1]])			#突出轮廓
  dst2=cv.filter2D(cat,-1,kernel2)
  
  cv.namedWindow("filter1",cv.WINDOW_NORMAL)
  cv.resizeWindow("filter1",3000,600)
  cv.imshow("filter1",np.hstack((dst2,cat,dst1)))
  cv.waitKey(0)
  cv.destroyAllWindows()
  ```



### 方盒滤波

![](https://gitee.com/genggenggenga/Picture/raw/master/images/image-20220926165130439.png)

- boxFilter(**src,ddepth,ksize**,dst,anchor,normalize,borderType)                       	#filter：滤波器
  - src：输入图像
  - ddepth：输出图像深度，-1表示与原图像一样，一般都是-1
  - ksize：卷积核大小，是一个元组
  - dst：输出图像
  - anchor：锚点；默认值 Point(-1,-1) 表示锚点位于内核中央。
  - normalize：标识符，表示内核是否被其区域归一化。若normalize==True，a=1/(width*height)；否则为1。
  - borderType：用于推断图像外部像素的某种边界模式。有默认值BORDER_DEFAULT

```python
#方盒滤波
box_filter=cv.boxFilter(cat,-1,(3,3),normalize=1)
cv.imshow("boxFilter",box_filter)
#cv.waitKey(0)
#cv.destroyAllWindows()
```



### 均值滤波

- blur(**src, ksize**, dst=None, anchor=None, borderType=None)				#blur：模糊
  - src：输入图像
  - ksize：卷积核大小，是一个元组
  - dst：输出图像
  - anchor：锚点；默认值 Point(-1,-1) 表示锚点位于内核中央。
  - borderType：用于推断图像外部像素的某种边界模式。有默认值BORDER_DEFAULT

```python
#均值滤波
blur_filter=cv.blur(cat,(3,3))
cv.imshow("blur",blur_filter)
cv.waitKey(0)
cv.destroyAllWindows()
```



### 高斯滤波

资料参考 [高斯滤波](https://blog.csdn.net/godadream/article/details/81568844)

​				[高斯滤波带图像](https://blog.csdn.net/qq_45832961/article/details/122351534)

- GaussianBlur(**src,ksize,sigmaX**,dst,sigmaY,borderType)
  - ksize：卷积核大小，是一个元组
  - sigmaX：X轴的标准差
  - sigmaY：Y轴标准差，默认为0，此时sigmaX=sigmaY
  - sigma越大，差别越小，平滑越明显
  - 如果sigmaX和sigmaY都设置为0，则会根据ksize计算sigma，ksize越大平滑越明显
  

```python
#高斯滤波
gaus_cat1=cv.GaussianBlur(cat,(9,9),100)
gaus_cat2=cv.GaussianBlur(cat,(21,21),sigmaX=0)

cv.namedWindow("window",cv.WINDOW_NORMAL)
cv.resizeWindow("window",2500,400)
cv.imshow("window",np.hstack((cat,gaus_cat1,gaus_cat2)))
cv.waitKey(0)
cv.destroyAllWindows()
```



### 中值滤波

取区域内的中位数作为卷积后的结果，去除椒盐噪音时效果非常好。

- medianBlur(src,ksize)
  - ksize不是元组，是单个数字

```python
img=cv_imread("D:\桌面\我的小车\中值滤波.png")
median_img=cv.medianBlur(img,5)
cv.imshow("window",np.hstack((img,median_img)))
cv.waitKey(0)
cv.destroyAllWindows()
```



![image-20220926200034816](https://gitee.com/genggenggenga/Picture/raw/master/images/image-20220926200034816.png)



### 双边滤波

资料参考 [手写双边滤波算法](https://blog.csdn.net/u013066730/article/details/87859184)

[				双边滤波数学原理](https://blog.csdn.net/qq_43144313/article/details/121874933)

​				[双边滤波](https://blog.csdn.net/qq_42593411/article/details/123411794)

​				

- bilateralFilter(**src,d,sigmaColor,sigmaSpace**,dst,borderType）

  - src：输入图像
  - d： 在滤波时选取的空间距离参数（直径）。当滤波器的直径大于5时，函数的运行速度会变慢，因此如果需要在实时系统中使用该函数，建议将滤波器的半径设置为5，对于离线处理含有大量噪声的滤波图像时，可以将滤波器的半径设为9。
  - sigmaColor：颜色空间的滤波的标准差
  - sigmaSpace：空间坐标的滤波的标准差
  - borderType：像素边界外推法标志


```python
#双边滤波
img=cv_imread(r"D:\桌面\屏幕截图.jpg")

def bilateral_callback(value):
    print(value)

cv.namedWindow("bilateral")
cv.createTrackbar("dsize","bilateral",1,21,bilateral_callback)
cv.createTrackbar("color_tracebar","bilateral",0,200,bilateral_callback)
cv.createTrackbar("space_tracebar","bilateral",0,200,bilateral_callback)
while 1:
    d=cv.getTrackbarPos("dsize","bilateral")
    color=cv.getTrackbarPos("color_tracebar","bilateral")
    space=cv.getTrackbarPos("space_tracebar","bilateral")

    bilateral_img=cv.bilateralFilter(img,d,color,space)
    cv.imshow("bilateral",np.hstack((img,bilateral_img)))
    if cv.waitKey(1)==ord("q"):
        break
cv.destroyAllWindows()
```



## 边缘检测

### Sobel算子

参考资料 [Sobel算子原理](https://blog.csdn.net/qq_37124237/article/details/82183177)

- Sobel(**src, ddepth, dx, dy**, dst, ksize=3, scale, delta, borderType)
  - dx、dy：求导的阶数，0表示这个方向上没有求导，一般为0、1、2。
  - ddepth：图像的深度，-1表示采用的是与原图像相同的深度。目标图像的深度必须大于等于原图像的深度；
  - ksize：指Sobel算子的大小，即卷积核的大小，必须为奇数1、3、5、7，默认为3。
    注意：如果ksize = -1，就演变为3*3的Scharr算子。

```python
#sobel算子
sobel_img=cv_imread("D:\桌面\我的小车\Image\Sobel.jpg")
dx=cv.Sobel(sobel_img,-1,1,0)
dy=cv.Sobel(sobel_img,-1,0,1)
abs_dx=cv.convertScaleAbs(dx)
abs_dy=cv.convertScaleAbs(dy)

new_abs=cv.add(abs_dx,abs_dy)

cv.imshow("window",np.hstack((sobel_img,abs_dx,abs_dy,new_abs)))
cv.waitKey(0)
cv.destroyAllWindows()
```



### Scharr算子

参考资料 [Scharr算子](https://blog.csdn.net/pengchengliu/article/details/108073375)

- Scharr(src,ddepth,dx,dy)
- Sobel(**src, ddepth, dx, dy**, ksize=-1)

```python
#sobel算子和scharr算子
cv.namedWindow("window1",cv.WINDOW_NORMAL)
cv.namedWindow("window2",cv.WINDOW_NORMAL)
cv.resizeWindow("window1",1600,400)
cv.resizeWindow("window2",1600,400)

sobel_img=cv_imread("D:\桌面\我的小车\Image\lena.jpg")
dx=cv.Sobel(sobel_img,-1,1,0)
dy=cv.Sobel(sobel_img,-1,0,1)
scharr_dx=cv.Scharr(sobel_img,-1,1,0)
scharr_dy=cv.Scharr(sobel_img,-1,0,1)

abs_dx=cv.convertScaleAbs(dx)
abs_dy=cv.convertScaleAbs(dy)
abs_scharr_dx=cv.convertScaleAbs(scharr_dx)
abs_scharr_dy=cv.convertScaleAbs(scharr_dy)

sobel=cv.add(abs_dx,abs_dy)
scharr=cv.add(abs_scharr_dx,abs_scharr_dy)

cv.imshow("window1",np.hstack((sobel_img,abs_dx,abs_dy,sobel)))
cv.imshow("window2",np.hstack((sobel_img,abs_scharr_dx,abs_scharr_dy,scharr)))
cv.waitKey(0)
cv.destroyAllWindows()
```





### 拉普拉斯算子

参考资料 [拉普拉斯算子](https://blog.csdn.net/weixin_44237705/article/details/108398323)

- Laplacian(src, ddepth, ksize, scale, delta, borderType)
  - ddepth：参数是图像的深度，-1表示采用的是与原图像相同的深度。目标图像的深度必须大于等于原图像的深度；
  - ksize：算子的大小，必须为1、3、5、7。默认为1
  - scale：缩放导数的比例常数，默认情况下没有伸缩系数
  - borderType：判断图像边界的模式。这个参数默认值为cv2.BORDER_DEFAULT

一般流程：

- 高斯模糊-去噪点`GaussianBlur()`
- 转换为灰度图像`cvtColor()`
- 拉普拉斯-二阶导数计算`Laplancian()`
- 取绝对值`convertScaleAbs()`
- 显示结果

```python
#拉普拉斯算子
gauss=cv.GaussianBlur(lena,(3,3),5)
gauss_laplace=cv.Laplacian(lena,-1,ksize=3)
gauss_laplace=cv.convertScaleAbs(gauss_laplace)
cv.imshow("window",np.hstack((lena,gauss_laplace)))
cv.waitKey(0)
cv.destroyAllWindows()
```



### Canny边缘检测

参考资料[Canny边缘检测](https://blog.csdn.net/m0_43609475/article/details/112775377)

​				[Canny图像处理算法](	https://blog.csdn.net/peter_z96/article/details/104429385)

- Canny(image, threshold1, threshold2[, edges[, apertureSize[, L2gradient ]]]) 
  - 其中较大的阈值2用于检测图像中明显的边缘，但一般情况下检测的效果不会那么完美，边缘检测出来是断断续续的。所以这时候用较小的第一个阈值用于将这些间断的边缘连接起来。
  
  - apertureSize：Sobel算子的大小
  
  - threshold1，threshold2：使用两个值域来检测边缘。一般情况下，使用一个阀值来检测边缘，但是这样做未免太武断了。如果能够使用启发式的方法确定一个上阀值和下阀值，位于下阀值之上的都可以作为边缘，这样就可能提高准确度。它设置两个阀值（threshold），分别为maxVal和minVal。其中大于maxVal的都被检测为边缘，而低于minval的都被检测为非边缘。对于中间的像素点，如果与确定为边缘的像素点邻接，则判定为边缘；否则为非边缘。
  
    ![img](https://gitee.com/genggenggenga/Picture/raw/master/images/20200717143852573.png)
  
  
    
  
  - L2gradient：一个布尔值，如果为真，则使用更精确的L2范数进行计算（即两个方向的倒数的平方和再开放），否则使用L1范数（直接将两个方向导数的绝对值相加）。

```python
#canny边缘检测
def callback(value):
    print(value)
cv.namedWindow("window")
cv.createTrackbar("minVal","window",0,255,callback)
cv.createTrackbar("maxVal","window",0,255,callback)
lena=cv.cvtColor(lena,cv.COLOR_BGR2GRAY)

while 1:
    minval=cv.getTrackbarPos("minVal","window")
    maxval=cv.getTrackbarPos("maxVal","window")

    new_lena=cv.GaussianBlur(lena,(5,5),3)
    final_lena=cv.Canny(new_lena,minval,maxval)
    cv.imshow("window",np.hstack((new_lena,final_lena)))
    if cv.waitKey(1)==ord("q"):
        break
cv.destroyAllWindows()
```





## 形态学

[形态学](https://blog.csdn.net/weixin_56197703/article/details/124018520)

### 全局二值化

- **thresh, dst = cv2.threshold(src, thresh, maxval, type)**

- src:输入图，只能输入单通道图像，通常来说为灰度图. dst:输出图

- thresh:阈值

- maxval:当像素值超过了阈值_(或者小于阈值，根据type来决定)，所赋予的值

- type:二值化操作的类型，包含以下5种类型: cv2.THRESH_BINARY;  cv2.THRESH_BINARY_INV;  c2.THRESH_TRUNC; cv2.THRESH_ TOZERO; cv2.THRESH_TOZERO_INV

  ```python
  cv2.THRESH_BINARY        #超过阈值部分取maxval(最大值)，否则取0 
  cv2.THRESH_BINARY_INV    #THRESH_BINARY的反转
  cv2.THRESH_TRUNC         #大于阈值部分设为阈值，否则不变
  cv2.THRESH_TOZERO        #大于阈值部分不改变，否则设为0
  cv2.THRESH_TOZERO_INV    #THRESH_TOZERO的反转
  ```



###  自适应阈值二值化

- adaptiveThreshold(src, maxValue, adaptiveMethod, thresholdType, blockSize, C)
  - maxValue：像素值上限
  - adaptive Method：自适应方法
    - cv2.ADAPTIVE_THRESH_MEAN_C ：区域内均值
    - cv2.ADAPTIVE_THRESH_GAUSSIAN_C ：区域内像素点加权和，权重为一个高斯窗口
  - blockSize：邻域大小，整型

当光线不好时效果不好

```python
#自适应阈值二值化
gray_lena=cv.cvtColor(lena,cv.COLOR_BGR2GRAY)
new_lena=cv.adaptiveThreshold(gray_lena,255,cv.ADAPTIVE_THRESH_MEAN_C,cv.THRESH_BINARY,3,0)
cv_imshow("window",new_lena)
```

![image-20221013113056277](https://gitee.com/genggenggenga/Picture/raw/master/images/image-20221013113056277.png)

###  

### 获取形态学卷积核

opencv提供了获取卷积核的api.不需要我们手工创建卷积核

- cv2.getStructuringElement(**shape, ksize**[, anchor])
  - shape是指卷积核的形状, 注意不是指长宽, 是指卷积核中1形成的形状.	
    - MORPH_RECT 卷积核中的1是矩形, 常用.
    - MORPH_ELLIPSE 卷积核中的1椭圆
    - MORPH_CROSS 卷积核中的1十字
    - ksize 卷积核的大小，元组形式



### 腐蚀与膨胀

参考资料[腐蚀与膨胀](https://blog.csdn.net/qq_40855366/article/details/81177174)

图像形态学中两种最基本的操作就是对图形的腐蚀和膨胀，可以说，形态学中的中高级操作都是建立在这两种操作之上。通过这两种基本的运算可以去除图像中的噪声，分割出独立的区域或者将两个区域连接在一起。

#### 腐蚀（求局部最小值）

- erode(**src,kernel**[, dst[, anchor[, iterations[, borderType[, borderValue]]]]])

```python
#腐蚀
cv.namedWindow("window",cv.WINDOW_NORMAL)
cv.resizeWindow("window",(520,360))
img=cv_imread("D:\桌面\我的小车\Image\腐蚀.jpg")
kernel=np.ones((3,3))
new_img=cv.erode(img,kernel,iterations=2)
cv_imshow("window",np.hstack((img,new_img)))
```



#### 膨胀（求局部最大值）

- cv2.dilate(**src, kernel**[, dst[, anchor[, iterations[, borderType[, borderValue]]]]])

```python
#膨胀
cv.namedWindow("window",cv.WINDOW_NORMAL)
cv.resizeWindow("window",(520,360))
img=cv_imread("D:\桌面\我的小车\Image\腐蚀.jpg")
kernel=cv.getStructuringElement(cv.MORPH_RECT,(3,3))
new_img=cv.dilate(img,kernel,iterations=3)
cv_imshow("window",np.hstack((img,new_img)))
```



### 开运算与闭运算

#### 开运算

**开运算 = 腐蚀 + 膨胀**

- 适用于消除图像**外部**的噪声

- cv2.morphologyEx(**src, op, kernel**[, dst[, anchor[, iterations[, borderType[, borderValue]]]]])
  - op 操作类型（operations），开运算是`cv2.MORPH_OPEN`
  - kernel ：卷积核
  - anchor 是锚点
  - iterations 是操作的迭代次数, 次数越多, 操作执行的次数越多, 效果越明显
  - borderType 边界类型
  - borderValue 边缘值



#### 闭运算

**闭运算 = 膨胀 + 腐蚀（先膨胀再腐蚀）**

- 适用于消除图像**内部**的噪声

- cv2.morphologyEx(src, op, kernel[, dst[, anchor[, iterations[, borderType[, borderValue]]]]])
  - op 操作类型（operations），闭运算是`cv2.MORPH_CLOSE`



### 形态学梯度

**梯度 = 原图 - 腐蚀**
**腐蚀之后原图边缘变小了, 原图 - 腐蚀 就可以得到腐蚀掉的部分, 即边缘**

- cv2.morphologyEx(**src, op, kernel**[, dst[, anchor[, iterations[, borderType[, borderValue]]]]])
  - op 操作类型（operations），梯度是`cv2.MORPH_GRADIENT`

```python
#梯度
cv.namedWindow("window",cv.WINDOW_NORMAL)
cv.resizeWindow("window",(720,360))
img=cv_imread("D:\桌面\我的小车\Image\腐蚀.jpg")
kernel=cv.getStructuringElement(cv.MORPH_RECT,(3,3))
new_img=cv.morphologyEx(img,cv.MORPH_OPEN,kernel,iterations=2)
final_img=cv.morphologyEx(new_img,cv.MORPH_GRADIENT,kernel,iterations=1)
cv_imshow("window",np.hstack((img,new_img,final_img)))
```

![image-20221004173745961](https://gitee.com/genggenggenga/Picture/raw/master/images/image-20221004173745961.png)

### 顶帽操作（tophat）

**顶帽 = 原图 - 开运算**
**开运算的效果是去除图像外的噪声, 原图 - 开运算就得到了去掉的噪声**

- morphologyEx(src, op, kernel[, dst[, anchor[, iterations[, borderType[, borderValue]]]]])
  - op 操作类型（operations），开运算是`cv2.MORPH_TOPHAT`

<img src="https://gitee.com/genggenggenga/Picture/raw/master/images/image-20221004173817290.png" alt="image-20221004173817290" style="zoom:50%;" />



### 黑帽操作（blackhat）

**黑帽 = 原图 - 闭运算**
**闭运算可以将图形内部的噪声去掉, 那么原图 - 闭运算的结果就是图形内部的噪声**

- morphologyEx(**src, op, kernel**[, dst[, anchor[, iterations[, borderType[, borderValue]]]]])
  - op 操作类型（operations），黑帽操作是cv2.MORPH_BLACKHAT

<img src="https://gitee.com/genggenggenga/Picture/raw/master/images/image-20221004174224359.png" alt="image-20221004174224359" style="zoom:50%;" />





## 图像轮廓

### 什么是图像轮廓

图像轮廓是具有相同颜色或灰度的连续点的曲线. 轮廓在形状分析和物体的检测和识别中很有用。

轮廓的作用:

- 用于图形分析
- 物体的识别和检测

注意:

- 为了检测的准确性，需要先对图像进行二值化或Canny操作。
- 在OpenCV中，找到轮廓就像从黑色背景中找到白色物体。因此请记住，要找到的对象应该是白色，背景应该是黑色。
- 画轮廓时会修改输入的图像, 如果之后想继续使用原始图像，应该将原始图像储存到其他变量中。



### 查找轮廓

- findContours(**image, mode, method**[, contours[, hierarchy[, offset]]])

  - **返回值：contours，hierarchy ，即轮廓（旧版本是list形式，新版本是tuple形式。tuple里是一个ndarray）和层级**
  - image：寻找轮廓的图像（单通道图像矩阵，可以是灰度图，但更常用的是二值图像，一般是经过Canny、拉普拉斯等边缘检测算子处理过的二值图像）


  - mode：查找轮廓的模式

      - RETR_EXTERNAL , 表示只检测最外围轮廓
      - RETR_LIST , 检测的轮廓不建立等级关系, 即检测所有轮廓
      - RETR_CCOMP , 每层最多两级, 从小到大, 从里到外，很少用
      - **RETR_TREE**, 按照树型存储轮廓, 从外到里，从大到小, 从右到左，最常用！


![image-20211117145126614](https://gitee.com/genggenggenga/Picture/raw/master/images/e4e8eea6b65644dcb6305ce14fc3fc78.png)


  - 
    method 轮廓近似方法也叫ApproximationMode

    - **CHAIN_APPROX_NONE**：保存所有轮廓上的点
    - **CHAIN_APPROX_SIMPLE**：只保存角点, 比如四边形, 只保留四边形的4个角, 存储信息少, 比较常用。
      返回值 ：contours和hierarchy 即轮廓（旧版本是list形式，新版本是tuple形式）和层级



  - contours：轮廓点。元组格式（不是ndarray），每一个元素为一个3维数组（其形状为（n,1,2），其中n表示轮廓点个数，2表示像素点坐标）,表示一个轮廓。


  - hierarchy：轮廓间的层次关系,为三维数组，形状为（1,n,4），其中n表示轮廓总个数，4指的是用4个数表示各轮廓间的相互关系。第一个数表示同级轮廓的下一个轮廓编号，第二个数表示同级轮廓的上一个轮廓的编号，第三个数表示该轮廓下一级轮廓的编号，第四个数表示该轮廓的上一级轮廓的编号。


  - offset：轮廓点的偏移量，格式为tuple,如（-10，10）表示轮廓点沿X负方向偏移10个像素点，沿Y正方向偏移10个像素点。



### 绘制轮廓

- cv2.drawContours(**image, contours, contourIdx, color**[, thickness[, lineType[, hierarchy[, maxLevel[, offset]]]]])
  - **直接绘制轮廓时会对原图进行绘制，建议拷贝一份新的图像**
  - image：指明在哪幅图像上绘制轮廓**（只有三通道的图像才显示轮廓）**
  - contours ：轮廓点
  - contourIdx ：要绘制的轮廓的编号。-1 表示绘制所有轮廓，0代表绘制最外面的轮廓，1代表绘制第二个轮廓（从外到内数），2代表绘制第三个轮廓（从外到内数），以此类推
  - color：轮廓的颜色（BGR）, 如 (0, 0, 255)表示红色
  - thickness：线宽。 -1 表示全部填充，1代表线的宽度，1、2、3…线宽依次变粗
  - lineType （可选参数）轮廓线型，包括cv2.LINE_4，cv2.LINE_8（默认），cv2.LINE_AA，分别表示4邻域线，8领域线，抗锯齿线（可以更好地显示曲线）
  - hierarchy （可选参数）层级结构，上述函数cv2.findContours()的第二个返回值，配合maxLevel参数使用
  - maxLevel （可选参数）等于0表示只绘制指定的轮廓，等于1表示绘制指定轮廓及其下一级子轮廓，等于2表示绘制指定轮廓及其所有子轮廓
  - offset （可选参数）轮廓点的偏移量



```python
#查找与绘制轮廓
gray=cv.cvtColor(hand,cv.COLOR_BGR2GRAY)
thresh,binary_hand=cv.threshold(gray,70,255,cv.THRESH_BINARY)
contours,hierarchy=cv.findContours(binary_hand,cv.RETR_TREE,cv.CHAIN_APPROX_SIMPLE)
hand_copy=hand.copy()
cv.drawContours(hand_copy,contours,0,(0,255,0),2)
cv_imshow("window",hand_copy)
```



### 轮廓面积和周长

#### 面积

轮廓面积是指每个轮廓中所有的像素点围成区域的面积，单位为像素。

轮廓面积是轮廓重要的统计特性之一，通过轮廓面积的大小可以进一步分析每个轮廓隐含的信息，例如通过轮廓面积区分物体大小识别不同的物体。

在查找到轮廓后, 可能会有很多细小的轮廓, 我们可以通过轮廓的面积进行过滤.

计算轮廓面积（不适用于具有自交点的轮廓，即重合），通常搭配findContours()函数使用。

- contourArea(**contour**[, oriented])

  - contour：输入的图像轮廓点，要用**contours[index]**。

  - oriented：有方向的区域标志，表示某一方向上的轮廓的面积值

    - 默认为False，表示返回不带方向的绝对值

    - True：依赖轮廓的方向（顺时针或逆时针），返回一个已标记区域的值

#### 周长

- arcLength(**curve, closed)**

  - curve：轮廓


  - closed：是否是闭合的轮廓

    - True：表示计算闭合的轮廓的周长
    
    - False：表示计算不是闭合的轮廓的周长，此时的周长比闭合时少最后一条
    
    - 例如，计算正方形的周长时，用True时计算四条边的和；用False时计算三条边的和

### 多边形逼近

> findContours后的轮廓信息contours可能过于复杂不平滑，可以用approxPolyDP()对该多边形曲线做适当近似,这就是轮廓的多边形逼近.
>
> Douglas-Peucker算法：
>
> 1. 从轮廓中挑出两个最远的点，进行相连；
> 2. 然后再原轮廓上寻找一个离线段距离最远的点，将该点加入逼近后的新轮廓，即连接着三个点形成的三角型作为轮廓；
> 3. 选择三角形的任意一条边出发，进行步骤2，将距离最远点加入新轮廓，直至满足输出的精度要求。

- approxPolyDP(**curve, epsilon, closed**[, approxCurve])
  - **返回值为一个ndarray，与cv2.drawContours搭配使用时，要把输出的approx转换成==list==类型传入到contours**

  - curve :近似逼近的轮廓，类型是mat（即ndarray），**contoursp[index]**

  - epsilon:判断点到相对应的line segment的距离，即DP算法使用的阈值（距离大于此阈值则舍弃，小于此阈值则保留，epsilon越小，折线的形状越“接近”曲线。）经过测试，该阈值设为0.02*perimeter比较合适。

  - closed轮廓是否闭合

    - True 近似曲线是闭合的(它的第一个和最后一个顶点是连接的)

    - False 表示不闭合


```python
#多边形逼近
gray=cv.cvtColor(hand,cv.COLOR_BGR2GRAY)
thresh,binary_hand=cv.threshold(gray,70,255,cv.THRESH_BINARY)
contours,hierarchy=cv.findContours(binary_hand,cv.RETR_TREE,cv.CHAIN_APPROX_NONE)
appro_hand=cv.approxPolyDP(contours[0],5,True)
copy_hand=hand.copy()
cv.drawContours(copy_hand,[appro_hand],0,(0,255,0),3)
cv_imshow("window",copy_hand)
```



### 凸包

> 逼近多边形是轮廓的高度近似，但是有时候，我们希望使用一个多边形的凸包来简化它。凸包跟逼近多边形很像，只不过它是物体最外层的凸多边形。凸包指的是完全包含原有轮廓，并且仅由轮廓上的点所构成的多边形。凸包的每一处都是凸的，即在凸包内连接任意两点的直线都在凸包的内部。在凸包内，任意连续三个点的内角小于180°。寻找图像的凸包，能够让我们做一些有意思的事情，比如手势识别等（简单来说外部凸起来的点连起来就是凸包）。

- convexHull(**points**[, hull[, clockwise[, returnPoints]]])
  - points：原始轮廓，不要传入逼近的轮廓
  - hull 输出凸包结果（ndarray）与cv2.drawContours搭配使用时，要把输出的hull转换成list类型传入到contours
  - colckwise：转动方向，默认为True，为顺时针绘制，反之为逆时针绘制
  - returnPoints：默认为True，返回凸包上点的坐标，如果设置为False，会返回与凸包点对应的轮廓上的点。

```python
#凸包
gray=cv.cvtColor(hand,cv.COLOR_BGR2GRAY)
thresh,binary_hand=cv.threshold(gray,150,255,cv.THRESH_BINARY)
contours,hierarchy=cv.findContours(binary_hand,cv.RETR_TREE,cv.CHAIN_APPROX_NONE)
copy_hand=hand.copy()
hull=cv.convexHull(contours[0])
cv.drawContours(copy_hand,[hull],0,(0,255,0),2)
cv_imshow("window",copy_hand)
```



### 外接矩形

外接矩形分为最小外接矩形和最大外接矩形

#### 最小外接矩形

- minAreaRect(points) 最小外接矩阵（存在旋转）
  - points ：轮廓（contours[index]）
  - 第一类返回值：(x, y), (w, h), angle = cv2.minAreaRect(**contours[0]**)
    -  内容是一个旋转矩形(RotatedRect)的参数: 矩形的起始坐标(x,y), 矩形的宽度和高度, 矩形的旋转角度.
  - 第二类返回值：rect= cv2.minAreaRect(**contours[0]**)
    - 通过**box=cv2.boxPoints（rect[, points]）**来自动获得矩形的四个顶点坐标
    - points 返回矩形的四个坐标（二维的ndarray）
    - 像素是通过整数的ndarray储存的，但是上面获得的box有可能是小数，我们要把小数位通过四舍五入重新算出来，再传入到countours，具体方法是box = np.round(box).astype(‘int64’)
  - 我们获得坐标后（用box接收），可以通过cv2.drawCounter( )将最小外接矩形画出来，但是通过cv2.drawCounters( )绘制外接矩形时要把box转化成list传入到counters参数里才能使用

```python
#最小外接矩形
gray=cv.cvtColor(hand,cv.COLOR_BGR2GRAY)
thresh,binary_hand=cv.threshold(gray,150,255,cv.THRESH_BINARY)
contours,hierarchy=cv.findContours(binary_hand,cv.RETR_TREE,cv.CHAIN_APPROX_NONE)
rect=cv.minAreaRect(contours[0])
box=cv.boxPoints(rect)
box=np.round(box).astype("int64")
copy_hand=hand.copy()
cv.drawContours(copy_hand,[box],0,(0,255,0),2)
cv_imshow("window",copy_hand)
```



#### 最大外接矩形

- cv2.boundingRect(**points**) 最大外接矩阵（方方正正的矩形，不存在旋转）
  - points 即为轮廓（contours[0]）
  - 返回四个参数，起始坐标（x,y）,宽高（w,h），用x，y，w，h接收，起始坐标在左下角，通过cv2.rectangle( )将最大外接矩形画出来

```python
#最大外接矩形
gray=cv.cvtColor(hand,cv.COLOR_BGR2GRAY)
thresh,binary_hand=cv.threshold(gray,150,255,cv.THRESH_BINARY)
contours,hierarchy=cv.findContours(binary_hand,cv.RETR_TREE,cv.CHAIN_APPROX_NONE)
x, y, w, h = cv.boundingRect(contours[0])
cv.rectangle(copy_hand, (x, y), (x + w, y + h), (255, 0, 0), 2)		#直接画在图上
cv_imshow("window",copy_hand)
```



### 图像金字塔

参考资料[图像金字塔](https://blog.csdn.net/weixin_56197703/article/details/124433973)

#### 高斯金字塔

向下取样

- cv2.pyrDown(**src**[, dst[, dstsize[, borderType]]]) 			

  - src 传入的图像

  - dst 输出图像

  - dstsize 输出图片的大小

  - borderType 边界类型

向上取样

- cv2.pyrUp(**src**[, dst[, dstsize[, borderType]]]

  - src 传入的图像

  - dst 输出图像

  - dstsize 输出图片的大小

  - borderType 边界类型



#### 拉普拉斯金字塔

<img src="https://gitee.com/genggenggenga/Picture/raw/master/images/afa149019bfde01fa5accde8a0bf86f7.png" alt="img" style="zoom: 33%;" />



## 图像直方图

参考资料[图像直方图](https://blog.csdn.net/weixin_56197703/article/details/124514058)





## 背景减除

- MOG：基于混合高斯进行背景建模
- MOG2：基于混合高斯进行背景建模，MOG的升级版本
- GMG：基于像素颜色进行背景建模，其主要依赖的是像素点的素值
- CNT：基于像素点计数进行背景建模，其主要依赖的是像素点相同数值的计数值
- KNN：基于K最近邻进行背景建模，根据附近颜色的相似度来进行的背景提取

### MOG2

该方法同样是使用基于高斯混合模型的背景前景分割算法。基于两篇论文：
Improved adaptive Gausian mixture model for background subtraction

Efficient Adaptive Density Estimation per Image Pixel for the Task of Background Subtraction

该算法的一个重要特征是它对每个像素点**选择了合适数量的高斯分布模型。**因此它也提供了一个在光照变换场景中更好的适应能力。



- **cv2.createBackgroundSubtractorMOG2**([, history[, varThreshold[, detectShadows]]])

  - history：Length of the history.历史时间长度

  - varThreshold：hreshold on the squared Mahalanobis distance between the pixel and the model.方差阈值，用于判断当前像素是前景还是背景。一般默认16，如果光照变化明显，如阳光下的水面，建议设为25,36，具体去试一下也不是很麻烦，值越大，灵敏度越低。

  - detectShadows：If true, the algorithm will detect shadows and mark them.如果为真，算法将检测阴影并标记它们。它会降低一旦速度，所以如果不需要这个特性，可设置成False.

在编写代码时，我们需要使用函数:**cv2.createBackgroundSubtractorMOG2()** 创建一个背景对象。这个函数有些可选参数，比如要进行建模场景的时间长度，阈值等。将他们全部设置为默认值。然后在整个视频中我们是需要使用backgroundsubtractor.apply() 就可以得到前景的掩膜了.

移动的物体会被标记为白色，背景会被标记为黑色的

```python
cv.namedWindow("window",cv.WINDOW_NORMAL)
cv.resizeWindow("window",300,500)

vc=cv.VideoCapture("D:\桌面\我的小车\Image\hand.mp4")
mog = cv.createBackgroundSubtractorMOG2()
while 1:
    ret,frame=vc.read()
    if ret:
        fgmask=mog.apply(frame)
        cv.imshow("window",fgmask)
    key=cv.waitKey(33)
    if key==ord("q"):
        break
vc.release()
cv.destroyAllWindows()
```



### KNN

- cv2.createBackgroundSubtractorKNN([, history[, dist2Threshold[, detectShadows]]])

  - history：Length of the history.历史时间长度

  - dist2Threshold：Threshold on the squared distance between the pixel and the sample to decide.whether a pixel is close to that sample. This parameter does not affect the background update.阈值由像素和样本之间的平方距离来决定。一个像素是否接近该样本。该参数不影响后台更新。

  - detectShadows：If true, the algorithm will detect shadows and mark them.如果为真，算法将检测阴影并标记它们。它会降低一旦速度，所以如果不需要这个特性，可设置成False.



### CNT

- cv2.bgsegm.createBackgroundSubtractorCNT([, minPixelStability[, useHistory[, maxPixelStability[, isParallel]]]])
  - minPixelStability：number of frames with same pixel color to consider stable.考虑稳定的相同像素颜色的帧数。
  - useHistory：determines if we’re giving a pixel credit for being stable for a long time.决定我们是否给一个像素信用，因为它是稳定的很长一段时间。
  - maxPixelStability：maximum allowed credit for a pixel in history.历史上一个像素的最大允许积分。
  - isParallel：determines if we’re parallelizing the algorithm.决定了我们是否在并行化算法。

总结：

如果追求速度的话，可以尝试使用CNT，MOG2，KNN。

如果追求质量的话，可以使用MOG2，KNN。

总的来说实际应用中，MOG2用的最多，KNN其次，CNT一般用于Raspberry Pi和多检测任务中。

```python
#实际应用
cv.namedWindow("window",cv.WINDOW_NORMAL)
cv.resizeWindow("window",1000,500)

vc=cv.VideoCapture("D:\桌面\我的小车\Image\车辆检测.mp4")
mog = cv.createBackgroundSubtractorMOG2(varThreshold=500)

min_width=70
min_height=50
line_height=600
offset=100
cnt=0

def centery(x,y,w,h):
    centery=h//2
    centery=centery+y
    return centery

while 1:
    ret,frame=vc.read()
    cv.line(frame, (0, line_height), (1800,line_height), (255, 0, 0), 5)
    if ret:
        gray=cv.cvtColor(frame,cv.COLOR_BGR2GRAY)
        #blur=cv.GaussianBlur(gray,(3,3),7)

        fgmask = mog.apply(gray)
        #kernel=cv.getStructuringElement(cv.MORPH_RECT,(3,3))
        #open_img=cv.morphologyEx(fgmask,cv.MORPH_OPEN,kernel,iterations=2)
        #close_fg=cv.morphologyEx(fgmask,cv.MORPH_CLOSE,kernel)
        contours,hierarchy=cv.findContours(fgmask,cv.RETR_TREE,cv.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            x,y,w,h=cv.boundingRect(contour)
            isValid= w>=min_width and h>=min_height
            if isValid:
                cv.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),5)
                #print(centery(x,y,w,h))
                if abs( centery(x,y,w,h)-line_height ) < offset:
                    print(cnt)
                    cnt=cnt+1


        cv.imshow("window",frame)
        key=cv.waitKey(33)
        if key==ord("q"):
            break
vc.release()
cv.destroyAllWindows()
```



## 目标追踪

参考链接[目标追踪](https://blog.csdn.net/weixin_56197703/article/details/124801441)

```python
cv.namedWindow("frame",cv.WINDOW_NORMAL)
cv.resizeWindow("frame",(1000,600))
# 初始化追踪器集合
trackers =cv.MultiTracker_create()
# 读取视频
cap = cv.VideoCapture("D:\桌面\我的小车\Image\车辆检测.mp4")

while True:
    ret, frame = cap.read()
    if not ret:
        break
    # 更新追踪器，追踪目标
    success, boxes = trackers.update(frame)
    # 绘制追踪到的矩形区域
    for box in boxes:
        # box是个浮点型ndarray, 画图需要整型
        (x, y, w, h) = [int(v) for v in box]
        cv.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

    cv.imshow('frame', frame)

    key = cv.waitKey(100)
    if key == ord('s'):
        # 框选ROI区域
        roi = cv.selectROI('frame', frame, showCrosshair=True, fromCenter=False)
        #         print(roi)
        # 创建一个实际的目标追踪器
        tracker = cv.TrackerCSRT_create()
        trackers.add(tracker, frame, roi)
    elif key == ord("q"):
        break

cap.release()
cv.destroyAllWindows()
```




# 模板函数

### 图片读取

```python
#读取图像，解决imread不能读取中文路径路径的问题
def cv_imread(file_path):
    #imdedcode读取的是RGB图像
    cv_img = cv.imdecode(np.fromfile(file_path,dtype=np.uint8),-1)
    return cv_img
```



### 图片显示

```python
def cv_imshow(name,img):
	cv.imshow(name,img)
    cv.waitKey(0)
    cv.destroyAllWindows()
```



# 为什么写到一半才发现o(╥﹏╥)o

https://blog.csdn.net/weixin_56197703/category_11720285.html
