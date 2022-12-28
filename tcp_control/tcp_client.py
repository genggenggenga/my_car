import pygame
import socket

recvStr=""
sendStr=""
cmd=""

#客户端
serverName="192.168.137.163"
serverPort=1113
clientSocket=socket.socket(socket.AF_INET,socket.SOCK_STREAM)



def tcp_client_connect():
    global clientSocket
    clientSocket.connect((serverName, serverPort))

def tcp_client_send(cmd):
    global clientSocket
    clientSocket.send(cmd.encode())

def tcp_client_send_close(cmd):
    global clientSocket
    clientSocket.send(cmd.encode())
    clientSocket.close()

def tcp_client_recv():
    global clientSocket
    recvStr=clientSocket.recv(1024)
    clientSocket.close()

def key2cmd(keys_pressed):
    global cmd
    if keys_pressed[pygame.K_UP] or keys_pressed[pygame.K_w]:
        cmd = "$up"
    elif keys_pressed[pygame.K_DOWN] or keys_pressed[pygame.K_s]:
        cmd = "$back"
    elif keys_pressed[pygame.K_LEFT] or keys_pressed[pygame.K_a]:
        cmd = "$left"
    elif keys_pressed[pygame.K_RIGHT] or keys_pressed[pygame.K_d]:
        cmd = "$right"
    elif keys_pressed[pygame.K_q]:
        cmd = "$spinLeft"
    elif keys_pressed[pygame.K_e]:
        cmd = "$spinRight"
    else:
        pass

try:
    while 1:
        keys_pressed=pygame.key.get_pressed()
        key2cmd(keys_pressed)
        tcp_client_send(cmd)

except:
    pass