# Chapter4 七彩探照灯

## 实验目的

远程登录树莓派后，运行程序，循环显示7种不同颜色的灯。

## 实验原理

树莓派的强大不仅在于它是一个卡式电脑，更重要的是引出的GPIO，可以通过编程使GPIO管脚输出高低电平。   

所谓的RGB三色灯和普通的LED灯其实没有什么不同，只是在封装上，RGB灯内封装了三个LED（红，绿，蓝），通过控制三种LED亮度（每个LED有256种亮度），可以混合出不同的颜色（256\*256*256种颜色）。

我们来瞅一瞅电路图，本实验中采用的RGB灯是共阴LED，一个引脚接地，其余的三个RGB引脚分别接在树莓派主控板上的Board编码15,13,18引脚上。我们只需在树莓派主控板上控制相应的引脚为高电平，即可点亮相应的LED。



![blob.png](https://gitee.com/genggenggenga/Picture/raw/master/images/1508929592102243.png)

​																									图一 树莓派接口

![blob.png](https://www.yahboom.com/Public/ueditor/php/upload/image/20171025/1508929751458804.png)

​																									图二 强光照明模块接口



## 代码编写

我们使用BCM编码来编写代码。

代码如下：

```python
# -*- coding:UTF-8 -*-

#引入GPIO库
import RPi.GPIO as GPIO            
import time

#RGB三色灯引脚对应的BCM
LED_R = 22
LED_G = 27
LED_B = 24

#设置RGB三色灯为BCM编码方式
GPIO.setmode(GPIO.BCM)

#RGB三色灯设置为输出模式
GPIO.setup(LED_R, GPIO.OUT)
GPIO.setup(LED_G, GPIO.OUT)
GPIO.setup(LED_B, GPIO.OUT)

#循环显示7种不同的颜色
try:
    while True:
        GPIO.output(LED_R, GPIO.HIGH)
        GPIO.output(LED_G, GPIO.LOW)
        GPIO.output(LED_B, GPIO.LOW)
        time.sleep(1)
        GPIO.output(LED_R, GPIO.LOW)
        GPIO.output(LED_G, GPIO.HIGH)
        GPIO.output(LED_B, GPIO.LOW)
        time.sleep(1)
        GPIO.output(LED_R, GPIO.LOW)
        GPIO.output(LED_G, GPIO.LOW)
        GPIO.output(LED_B, GPIO.HIGH)
        time.sleep(1)
        GPIO.output(LED_R, GPIO.HIGH)
        GPIO.output(LED_G, GPIO.HIGH)
        GPIO.output(LED_B, GPIO.LOW)
        time.sleep(1)
        GPIO.output(LED_R, GPIO.HIGH)
        GPIO.output(LED_G, GPIO.LOW)
        GPIO.output(LED_B, GPIO.HIGH)
        time.sleep(1)
        GPIO.output(LED_R, GPIO.LOW)
        GPIO.output(LED_G, GPIO.HIGH)
        GPIO.output(LED_B, GPIO.HIGH)
        time.sleep(1)
        GPIO.output(LED_R, GPIO.LOW)
        GPIO.output(LED_G, GPIO.LOW)
        GPIO.output(LED_B, GPIO.LOW)
        time.sleep(1)
except:
    print ("七彩探照灯出现异常！！")

    
#会执行gpio.cleanup()语句清除GPIO管脚的状态
GPIO.cleanup()
```

