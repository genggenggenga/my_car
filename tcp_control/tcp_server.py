# 服务端  
import socket  
from time import *

from car_run import *  
from led import *
from auto_drive import *
  
recvBuf=""  
sendBuf=""  
light_flag=0 
power_flag=0 
 
serverPort = 1113  
serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  
serverSocket.bind(("", serverPort))  
serverSocket.listen(1) 
print("The server is already to receive!")  
global connectionSocket  
connectionSocket, addr = serverSocket.accept() 
 
  
 
def tcp_server_recv():  
    global connectionSocket, recvBuf  
    recvBuf = connectionSocket.recv(50).decode() 
    print("receive successful:"+recvBuf) 
  
def tcp_server_recv_close():  
    global connectionSocket,recvBuf  
    connectionSocket, addr = serverSocket.accept() 
    print(addr) 
    recvBuf= connectionSocket.recv(50).decode() 
    print(recvBuf) 
    connectionSocket.close() 
    print("receive successful") 
  
def tcp_server_send(str):  
    global connectionSocket  
    serverSocket.send(str.encode())  
    connectionSocket.close()  
  
def tcp_server_parse(recvBuf):
    if recvBuf[0]=="$": 
        global light_flag 
        global power_flag
        if recvBuf.find("poweron")!=-1:
            power_flag=1 
            car_run_init() 
            led_init() 

        if power_flag==0:
            print("小车未开机") 
            return 

        #小车已开机 
        if recvBuf.find("car")!=-1: 
            car_run_init() 
            #行驶模块 
            if recvBuf.find("up")!=-1: 
                run(LOW_SPEED,LOW_SPEED) 
                sleep(0.5) 
            elif recvBuf.find("back")!=-1: 
                back(LOW_SPEED,LOW_SPEED) 
                sleep(0.5) 
            elif recvBuf.find("left")!=-1: 
                left(LOW_SPEED) 
                sleep(0.5) 
            elif recvBuf.find("right")!=-1: 
                right(LOW_SPEED) 
                sleep(0.5) 
            elif recvBuf.find("spinLeft")!=-1: 
                spin_left(SPIN_LOW_SPEED) 
                sleep(0.5) 
            elif recvBuf.find("spinRight")!=-1: 
                spin_right(SPIN_LOW_SPEED) 
                sleep(0.5)
            elif recvBuf.find("autodrive")!=-1:
                car_auto_drive()
            brake()
        #灯光模块 
        elif recvBuf.find("turn_on_light")!=-1: 
            led_init() 
            print("LED init") 
            light_flag=1 
        elif recvBuf.find("turn_off_light")!=-1: 
            led_clean() 
            light_flag=0 
        elif recvBuf.find("red")!=-1: 
            if light_flag: 
                LED_RED() 
            else: 
                print("LED未开，无法展示红色") 
        elif recvBuf.find("green")!=-1: 
            if light_flag: 
                LED_GREEN() 
            else: 
                print("LED未开，无法展示绿色") 
        elif recvBuf.find("blue")!=-1: 
            if light_flag: 
                LED_BLUE() 
            else: 
                print("LED未开，无法展示绿色") 
 
        #小车关机 
        elif recvBuf.find("poweroff")!=-1: 
            power_flag=0
            brake()
            car_run_clean() 
            led_clean() 
        else: 
            pass 
    else: 
        print("not a commond") 
         
try: 
    GPIO.setmode(GPIO.BCM) 
    GPIO.setwarnings(False) 
    #car_run_init() 
    while 1: 
        tcp_server_recv()  
        tcp_server_parse(recvBuf) 
        cmd="" 
  
except:  
    pass  
  
