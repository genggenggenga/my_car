# Chapter2 环境搭建

## 系统烧录
由于将树莓派结果手时，SD卡中已经烧录好了系统，所以再次不在赘述系统的烧录。如感兴趣可自行查询资料。

## 树莓派开机
树莓派提供了好多外设接口，可以连接显示器、鼠标以及显示器。通过显示器可以操作树莓派。
但是受限于我们手头的硬件资源，一般手头里就只有一个笔记本电脑，无法充当显示器，所以我们选择无显示屏连接。
### 网线连接
https://blog.csdn.net/wallace89/article/details/120473842?ops_request_misc

### 无网线连接

要想使用笔记本电脑对树莓派进行远程操作，尤其是我们要做的智能小车，必然是不能被网线束缚住的。所以我们要脱离开网线，远程对树莓派进行操作。
#### 远程桌面连接
既然可以使用有线网线进行连接，那么可以使用无线网吗？那是必然可以的。
我们可以使用树莓派连接笔记本的热点，通过网络进行连接。(笔记本连接下校园网就好)

1. 打开笔记本热点，查看用户名及密码。**将网络频段设置为2.4GHz。**
![开启笔记本热点](https://img-blog.csdnimg.cn/84020a082ca54410916e4eb41c29f489.png#pic_center)


2. 将SD插入读卡器，连接电脑。
  ![配置文件](https://img-blog.csdnimg.cn/e7b330fa26dc499396c958aab3ec296e.png#pic_center)

   1. 新建一个名字为"ssh"的文件，注意这个文件没有任何后缀（先新建一个文本文件，重命名为"ssh"）
   2. 新建一个名字为"wpa_supplicant.conf"的文件，内容如下：

   ```
   ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
   update_config=10
   country=CN
  
   network={
     ssid="热点名称"
     psk="热点密码"
     key_mgmt=WPA-PSK
   }
  
   ```
3. 将SD卡插回树莓派，连接电源，树莓派就会自动连接电脑热点了。
   但是个别同学可能会遇到连接不上的情况。(如果连接上了可忽略)

   此时要注意网络频段是否设置的2.4GHz，如果仍然不行的话就要查看网络设置了。

   打开 控制面板\网络和 Internet\网络连接，找到WLAN和显示“Microsoft Wi-Fi Direct Virtual Adapter”的本地连接。

   ![](https://img-blog.csdnimg.cn/ae78e9cd64a84d8aa90a921775f236fe.png#pic_center)

   双击WLAN，进入"属性"设置，进入“共享”，勾选所有，然后将家用网络连接设置为显示“Microsoft Wi-Fi Direct Virtual Adapter”的本地连接，点击确定。

   双击进入该本地连接，此时IPV4连接显示了Internet。
   ![](https://img-blog.csdnimg.cn/94a966dcc1c44d7d95ba64ffec14107e.png#pic_center)

   这时树莓派就可以连接到电脑热点了。

4. 远程桌面连接

   连接上热点后，就可以查看树莓派的IP地址了。

   ![](https://img-blog.csdnimg.cn/9f1959ebc1c944998e7abeb27c18dcb1.png#pic_center)


   下载软件“Putty”，输入刚刚记录的IP，选择SSH连接，点击Open,就进入了树莓派，输入用户名(默认为“pi”)和密码(默认为：raspberry)。此时就可以通过命令行控制树莓派了。

   ![](https://img-blog.csdnimg.cn/13c34d536eb2438b91e48448ab7dccad.png#pic_center)

   输入命令安装xrdp：
   ```linux
   sudo apt-get install xrdp
   ```

   ![](https://img-blog.csdnimg.cn/e05c558faeeb49efa751f27d6bcb96e3.png#pic_center)

   安装好后我们就可以使用windows自带的"远程桌面连接""来进入树莓派的图形界面了。在搜索框搜索到"远程桌面连接"，输入IP地址，连接到树莓派，就完成了与树莓派的远程连接！

   输入用户名和密码，就可以看到树莓派的霞光了哈哈。

   ![](https://img-blog.csdnimg.cn/d667138f70584e71b4868f7d22394580.png#pic_center)

   ![](https://img-blog.csdnimg.cn/99ef7e5bf3bd4e5eb1e034a63cc55f3e.png#pic_center)


   但其实还有一种方法可以对树莓派进行远程操作，那就是VNC连接。虽然说VNC连接没有远程桌面连接的清晰度高(看着太不舒服了)，但是有一个好处，就是可以非常方便地实现计算机和树莓派之间的文件传输。所以继搞下去吧。

   在树莓派中卡在终端，输入命令以打开vnc服务：
   ```linux
   $ vncserver
   ```


   在计算机中下载“vnc viewer” ，连接树莓派
   ![](https://img-blog.csdnimg.cn/253d125a77994728bba4a33e3caadc3c.png#pic_center)

   这样同样可以远程操控树莓派。至于文件传输呢，参见下面两个图片。

   - 计算机传树莓派：将鼠标移至屏幕上方，第五个按钮就是文件传输。

   ![](https://img-blog.csdnimg.cn/1c8cf0c632d94f11ab25e80e782c58cf.png#pic_center)

   - 树莓派传计算机：点击右上方的VNC图标。
      ![](https://img-blog.csdnimg.cn/b94b8423ef3f4a96b131cc2588de2fa2.png#pic_center)

   再提一嘴，这个ip地址每次连接的时候可能会变来变去，如果不想每次都查看ip地址的话，可以自行查询给树莓派设置静态ip地址，这样子ip地址就会一直保持一个啦。

# 收工

22/11/1：补充：Chapter16有一些小tips可以先看一下。
