# Chapter3 小车安装及必备基础知识

## 小车安装

配置好了我们的树莓派，我们就可以结合我们的外设进行编程了。

先安装好我们的小车(省略安装过程)。安装完毕后如下：

<img src="https://gitee.com/genggenggenga/Picture/raw/master/images/car.png" alt="car" style="zoom:50%;" />

此处省略我自己安装的小车(真的离谱，莫名其妙就显示不出来[○･｀Д´･ ○])

为了我们中途测试方便，可以先安装好第一层，最上边的可以用到之后再安装。



## 外设编程

接下来所有的外设编程我们都要用到一个库——WringPi。一般来讲树莓派是自带wringpi库的，我们可以树莓派中断中输入命令```$ gpio readall```来查看该库是否完好，但一般来说是会报错的。因为这个库停止更新了，只适配到树莓派3B，所以我们需要使用命令来重新下载。

```shell
# 选择要保存的目录，笔者是用户根目录下的Work
$ cd ~/Work
 
# 下载deb包
$ wget https://project-downloads.drogon.net/wiringpi-latest.deb
 
# 安装deb包
$ sudo dpkg -i wiringpi-latest.deb
```

如果命令能顺利执行的话那就太幸运啦，但在这个时候我没有顺利的执行，报出了错误，所以需要手动安装。经过一段时间的折磨，我发现可以直接在我们的计算机上直接访问```https://project-downloads.drogon.net/wiringpi-latest.deb``` ，会自动下载deb包，然后通过VNC传输到树莓派中哈哈，然后执行第三条命令就好了。

安装完毕后可以通过```gpio -v```来查看版本号，安装成功，``$ gpio readall``命令也能正常执行。

## 必备知识

在Wringpi中封装了好多我们会用到的函数，在python中同样提供了RPI库。

偷个小懒，大家可以前往https://blog.csdn.net/finedayforu/article/details/116562288去学习！！



