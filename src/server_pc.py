import socket
import threading
import time
import sys



flag = 0
global flag_11
flag_11 = 0
def cv_face():
    print('okok')

def socket_service():
    global flag_11
    flag_11 = 0
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # 防止socket server重启后端口被占用（socket.error: [Errno 98] Address already in use）
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(('192.168.1.25', 6666))
        s.listen(10)
    except socket.error as msg:
        print(msg)
        sys.exit(1)
    print('Waiting connection...')
    threads = []
    conn, addr = s.accept()
 
    while 1:  
        
        print('Accept new connection from {0}'.format(addr))
        conn.send(('Hi, Welcome to the server!').encode())
    

        data = conn.recv(1024)
        if data.decode()=='haha':
            print('a=aaa====')
            flag_11 = 1
            break
        print('{0} client send data is {1}'.format(addr, data.decode()))#b'\xe8\xbf\x99\xe6\xac\xa1\xe5\x8f\xaf\xe4\xbb\xa5\xe4\xba\x86'
        
        # time.sleep(1)
        if data == 'exit' or not data:
            print('{0} connection close'.format(addr))
            conn.send(bytes('Connection closed!'),'UTF-8')
            break
        conn.send(bytes('Hello, {0}'.format(data),"UTF-8"))#TypeError: a bytes-like object is required, not 'str'
    
    conn.close()
    return flag_11
            
        
        

        
# def deal_data(conn, addr):
    # global flag_11
    # print('Accept new connection from {0}'.format(addr))
    # conn.send(('Hi, Welcome to the server!').encode())
    # flag_11 = 0
    # while 1:
    #     data = conn.recv(1024)
    #     if data.decode()=='haha':
    #         print('a=aaa====')
    #         flag_11 = 1
    #         break
    #     print('{0} client send data is {1}'.format(addr, data.decode()))#b'\xe8\xbf\x99\xe6\xac\xa1\xe5\x8f\xaf\xe4\xbb\xa5\xe4\xba\x86'
        
    #     time.sleep(1)
    #     if data == 'exit' or not data:
    #         print('{0} connection close'.format(addr))
    #         conn.send(bytes('Connection closed!'),'UTF-8')
    #         break
    #     conn.send(bytes('Hello, {0}'.format(data),"UTF-8"))#TypeError: a bytes-like object is required, not 'str'
    # conn.close()
    # return flag_11
 
 



import cv2
import numpy as np
import os


socket_service()
if flag_11 == 1:
    cv_face()







