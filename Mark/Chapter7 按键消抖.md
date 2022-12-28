# Chapter7 按键消抖

## 实验目的

探究按键消抖原理，并运用到小车中。

## 实验原理

#### 消抖原理

> 对于机械弹性开关，当机械触点断开和闭合时，由于机械触点的弹性作用，按键开关在闭合时不会马上就稳定的接通，在断开时也不会一下子彻底断开，而是在闭合和断开的瞬间伴随了一连串的抖动；抖动时间一般都是由按键的机械特性决定的，一般都会在10ms以内。所以当检测到按键状态变化时，不是应该立即响应动作，而应先等待稳定后再进行处理，这就是按键消抖。消抖分为硬件消抖和软件消抖。
>
> ![blob.png](https://gitee.com/genggenggenga/Picture/raw/master/images/1508940128524127.png)
>
> ​																								按键抖动状态图
>
> 硬件消抖就是在按键上并联一个电容利用电容的充放电特性来对抖动过程中产生的电压毛刺进行平滑处理，从而实现消抖。
>
> ![image-20220919173027709](https://gitee.com/genggenggenga/Picture/raw/master/images/image-20220919173027709.png)
>
> 但实际应用中，这种方式的效果往往不是很好，而且还增加了成本和电路复杂度，所以实际中使用的并不多。绝大多数情况下，我们是用软件即程序来实现消抖的。
>
> 软件消抖指的是检测出键闭合后执行一个延时程序,产生5ms～10ms的延时,让前沿抖动消失后再一次检测键的状态,如果仍保持闭合状态电平,则确认为真正有键按下。当检测到按键释放后,也要给5ms～10ms的延时,待后沿抖动消失后才能转入该键的处理程序。

#### 硬件原理

![image-20220922185854973](https://gitee.com/genggenggenga/Picture/raw/master/images/image-20220922185854973.png)

没有按下K2时，引脚为高电平；按下之后，引脚接地，为低电平。

## 代码编写

```python
#小车按键定义
key = 8

#引脚初始化
GPIO.setup(key,GPIO.IN)

#按键检测
def key_scan():
    while GPIO.input(key):                  #等待按下
        pass
    while not GPIO.input(key):              #按下了要检验
        time.sleep(0.01)
        if not GPIO.input(key):             #还是低电平，则确实按下了
            time.sleep(0.01)    
            while not GPIO.input(key):      #等待松手
                pass
```

