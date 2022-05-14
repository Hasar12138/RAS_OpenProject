from sys import flags
import time
import cv2 as cv
from threading import Thread
from djitellopy import Tello
import torch
from time import sleep
from lidar_model import LidarModel
from wmr_model import KinematicModel
import cv2
import numpy as np
from utils import *
import cubic_spline
from PIL import Image
import timeit
import socket
import sys


class AStar():
    def __init__(self, m):
        self.map = m
        self.initialize()

    def initialize(self):
        self.queue = []
        self.parent = {}
        self.h = {}  # Distance from start to node
        self.g = {}  # Distance from node to goal
        self.goal_node = None

    # estimation
    def _distance(self, a, b):
        # Diagonal distance
        d = np.max([np.abs(a[0]-b[0]), np.abs(a[1]-b[1])])
        return d

    def planning(self, start=(100, 200), goal=(375, 520), inter=10, img=None):
        # Initialize
        self.initialize()
        self.queue.append(start)
        self.parent[start] = None
        self.g[start] = 0
        self.h[start] = self._distance(start, goal)
        node_goal = None
        while(1):
            min_dist = 99999
            min_id = -1
            for i, node in enumerate(self.queue):
            

                f = self.g[node]  # find distance from start to this node
                y = self.h[node]  # find distance from node to goal

                if f+y < min_dist:
                    min_dist = f+y
                    min_id = i

            # pop the nearest node
            p = self.queue.pop(min_id)

            # meet obstacle, skip
            if self.map[p[1], p[0]] < 0.5:
                continue

            # find goal
            if self._distance(p, goal) < inter:
                self.goal_node = p
                break

            # eight direction
            pts_next1 = [(p[0]+inter, p[1]), (p[0], p[1]+inter),
                         (p[0]-inter, p[1]), (p[0], p[1]-inter)]
            pts_next2 = [(p[0]+inter, p[1]+inter), (p[0]-inter, p[1]+inter),
                         (p[0]-inter, p[1]-inter), (p[0]+inter, p[1]-inter)]
            pts_next = pts_next1 + pts_next2

            for pn in pts_next:
 
                if pn not in self.parent:
                    self.queue.append(pn)
                    self.parent[pn] = p # store next point's parent is current p
                    self.g[pn] = self.g[p] + inter # estimation of next point g(v)
                    self.h[pn] = self._distance(pn, goal)

                elif self.g[pn] > self.g[p] + inter:
                    self.parent[pn] = p
                    self.g[pn] = self.g[p] + inter

            if img is not None:
                cv2.circle(img, (start[0], start[1]), 5, (0, 0, 1), 3)
                cv2.circle(img, (goal[0], goal[1]), 5, (0, 1, 0), 3)
                cv2.circle(img, p, 2, (0, 0, 1), 1)
                img_ = cv2.flip(img, 0)
                cv2.namedWindow('map', cv2.WINDOW_NORMAL)
                cv2.imshow("map", img_)
                k = cv2.waitKey(1)
                if k == 27:
                    break

        # Extract path
        path = []
        p = self.goal_node
        while(True):
            path.insert(0, p)
            if self.parent[p] == None:
                break
            p = self.parent[p]
        if path[-1] != goal:
            path.append(goal)
        return path


global save_path
global flag
global flag_fly
global now_pos

flag=0
flag_fly=0
tello = Tello()
tello.connect()


keepRecording = True
keepMoveing = True
tello.streamon()
frame_read = tello.get_frame_read()
# cv.waitKey(0)


def videoplayer():
    # cv.namedWindow("drone", 0)
    # cv.resizeWindow("drone", 1200, 1200) 
    global save_path
    global flag
    global flag_fly
    global flag_back
    global now_pos

    smooth = True
    # cv2.namedWindow("map", 0)
    # cv2.resizeWindow("map", 1500, 1500) 

    # a = cv2.imread("Maps/map1.png")
    # print(a)
    img = cv2.flip(cv2.imread("Maps/map5.jpeg"), 0)

    # cv2.namedWindow('map', cv2.WINDOW_NORMAL)
    cv2.imshow("map", img)


    img = cv2.resize(img,(687,1256))
    print(img.shape[:2])
    img[img > 128] = 255
    img[img <= 128] = 0
    m = np.asarray(img)
    m = cv2.cvtColor(m, cv2.COLOR_RGB2GRAY)
    m = m.astype(float) / 255
    # m = 1-cv2.dilate(1-m, np.ones((20, 20)))
    img = img.astype(float)/255

    lmodel = LidarModel(m)
    car = KinematicModel()
    # pos = (140,1000,0)
    pos = (450,400,0)
    car.x = pos[0]
    car.y = pos[1]
    car.yaw = pos[2]

    # start = (80, 700)
    # goal = (0, 330)
    # start = (140, 1000)
    # goal = (450, 400)

    # goal = (450, 200)
    goal = (130, 200)
    start = (450, 400)

    a = timeit.default_timer()
    astar = AStar(m)
    astar = AStar(1-cv2.dilate(1-m, np.ones((40, 40))))
    astar = AStar(1-cv2.dilate(1-m, np.ones((60, 60))))
    path = astar.planning(start=start, goal=goal, img=img, inter=40)
    save_path=path
    # print(path)
    b = timeit.default_timer()
    print("Time: ", b-a)

    cv2.circle(img, (start[0], start[1]), 5, (0, 0, 1), 3)
    cv2.circle(img, (goal[0], goal[1]), 5, (0, 1, 0), 3)
    # Extract Path
    if not smooth:
        for i in range(len(path)-1):
            cv2.line(img, path[i], path[i+1], (1, 0, 0), 2)
    else:
        path = np.array(cubic_spline.cubic_spline_2d(path, interval=1))
        for i in range(len(path)-1):
            cv2.line(img, cubic_spline.pos_int(path[i]), cubic_spline.pos_int(path[i+1]), (1, 0, 0), 1)

    flag=1
    flag_fly=1
    flag_jetbot=1
    images = []
    i=0
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect(('192.168.1.64', 6666))
    except socket.error as msg:
        print(msg)
        # sys.exit(1)
    # print(s.recv(1024))#目的在于接受：Accept new connection from (...
    while keepRecording:
        
        image = frame_read.frame
        car.update()
        pos = (car.x, car.y, car.yaw)
        sdata, plist = lmodel.measure_2d(pos)
        img_ = img.copy()
        for pts in plist:
            cv2.line(
                img_, 
                (int(1*pos[0]), int(1*pos[1])), 
                (int(1*pts[0]), int(1*pts[1])),
                (0.0,1.0,0.0), 1)
        
        img_ = car.render(img_)
        img_ = cv2.flip(img_,0)

        #Collision
        p1,p2,p3,p4 = car.car_box
        l1 = Bresenham(p1[0], p2[0], p1[1], p2[1])
        l2 = Bresenham(p2[0], p3[0], p2[1], p3[1])
        l3 = Bresenham(p3[0], p4[0], p3[1], p4[1])
        l4 = Bresenham(p4[0], p1[0], p4[1], p1[1])
        check = l1+l2+l3+l4
        collision = False
        for pts in check:
            if m[int(pts[1]),int(pts[0])]<0.5:
                collision = True
                car.redo()
                car.v = -0.5*car.v
                break
        cv2.imshow("map",img_)
        k = cv2.waitKey(1)
        car.x=now_pos[0]
        car.y=now_pos[1]
          
        key = cv.waitKey(1) & 0xff
        if key == 27: # ESC
            break

        cv.imshow("drone", image)
        
        if flag_back==1 and flag_jetbot==1:
            data = 'haha'.encode()
            s.send(data)
            print('aa',s.recv(1024))
            s.close()
            flag_jetbot=0


player = Thread(target=videoplayer)
player.start()

# flag_fly=0
global flag_back
flag_back = 0
now_pos=(450, 400)
i=1
j=1
drone_yaw=0
back_path=[]
while True:
    if flag==1:
        if flag_fly==1:
            tello.takeoff()
            tello.move_up(70)
            flag_fly=0
            print(flag_fly,'flag_fly')

            print(save_path)
            back_path=save_path[::-1]
            length=len(save_path)
        
        # flag=0
        if i < length:
            # sleep(1)

            if save_path[i][0]>now_pos[0]: 
                print('case1')
                if drone_yaw == 0:
                    tello.move_left(int((save_path[i][0]-now_pos[0])*1.0))
                    sleep(1)
                   
                elif drone_yaw == 180:
                    tello.move_right(int((save_path[i][0]-now_pos[0])*1.0))
                    sleep(1)

                if save_path[i][1]>now_pos[1]: 
                    print('case1_1')
                    if drone_yaw ==180:
                        pass
                    elif drone_yaw ==0:
                        drone_yaw =180
                        tello.rotate_clockwise(180)
                        sleep(1)
                    tello.move_forward(int((save_path[i][1]-now_pos[1])*1.05))
                    sleep(1)
                    
                elif save_path[i][1]<now_pos[1]:
                    print('case1_2')
                    if drone_yaw ==0:
                        pass
                    elif drone_yaw == 180:
                        drone_yaw =0
                        tello.rotate_clockwise(180)
                        sleep(1)
                    tello.move_forward(int((now_pos[1]-save_path[i][1])*1.05))    
                    sleep(1)
                
                now_pos=save_path[i]
                print(i,now_pos) 
                i+=1

            elif save_path[i][0]<now_pos[0]: 
                print('case2')
                if drone_yaw == 0:
                    tello.move_right(int((now_pos[0]-save_path[i][0])*1.0))
                    sleep(1)
                    
                elif drone_yaw == 180:
                    tello.move_left(int((now_pos[0]-save_path[i][0])*1.0))    
                    sleep(1) 

                if save_path[i][1]>now_pos[1]: 
                    print('case2_1')
                    if drone_yaw ==180:
                        pass
                    elif drone_yaw ==0:
                        drone_yaw =180
                        tello.rotate_clockwise(180)
                        sleep(1)
                    tello.move_forward(int((save_path[i][1]-now_pos[1])*1.05))
                    sleep(1)
                    
                elif save_path[i][1]<now_pos[1]:
                    print('case2_2')
                    if drone_yaw ==0:
                        pass
                    elif drone_yaw == 180:
                        drone_yaw =0
                        tello.rotate_clockwise(180)
                        sleep(1)
                    tello.move_forward(int((now_pos[1]-save_path[i][1])*1.05))    
                    sleep(1)
                
                now_pos=save_path[i]
                print(i,now_pos) 
                i+=1
            else:
                print('case3')
                if save_path[i][1]>now_pos[1]: 
                    print('case3_1')
                    if drone_yaw ==180:
                        pass
                    elif drone_yaw ==0:
                        drone_yaw =180
                        tello.rotate_clockwise(180)
                        sleep(1)
                    tello.move_forward(int((save_path[i][1]-now_pos[1])*1.05))
                    sleep(1)
                    
                elif save_path[i][1]<now_pos[1]:
                    print('case3_2')
                    if drone_yaw ==0:
                        pass
                    elif drone_yaw == 180:
                        drone_yaw =0
                        tello.rotate_clockwise(180)
                        sleep(1)
                    tello.move_forward(int((now_pos[1]-save_path[i][1])*1.05))    
                    sleep(1)
                
                now_pos=save_path[i]
                print(i,now_pos)  
                i+=1
           
        elif i == length:
            flag_back = 1
            print('====================back=====================')
            # break

    else:
        pass

    if flag_back==1:
        # =save_path[::-1]
        length_back=len(back_path)

        if j < length_back:
        # sleep(1)
            if back_path[j][0]>now_pos[0]: 
                print('case1')
                if drone_yaw == 0:
                    tello.move_left(int((back_path[j][0]-now_pos[0])*1.0))
                    sleep(1)
                
                elif drone_yaw == 180:
                    tello.move_right(int((back_path[j][0]-now_pos[0])*1.0))
                    sleep(1)

                if back_path[j][1]>now_pos[1]: 
                    print('case1_1')
                    if drone_yaw ==180:
                        pass
                    elif drone_yaw ==0:
                        drone_yaw =180
                        tello.rotate_clockwise(180)
                        sleep(1)
                    tello.move_forward(int((back_path[j][1]-now_pos[1])*1.10))
                    sleep(1)
                    
                elif back_path[j][1]<now_pos[1]:
                    print('case1_2')
                    if drone_yaw ==0:
                        pass
                    elif drone_yaw == 180:
                        drone_yaw =0
                        tello.rotate_clockwise(180)
                        sleep(1)
                    tello.move_forward(int((now_pos[1]-back_path[j][1])*1.10))    
                    sleep(1)
                
                now_pos=back_path[j]
                print(j,now_pos) 
                j+=1

            elif back_path[j][0]<now_pos[0]: 
                print('case2')
                if drone_yaw == 0:
                    tello.move_right(int((now_pos[0]-back_path[j][0])*1.0))
                    sleep(1)
                    
                elif drone_yaw == 180:
                    tello.move_left(int((now_pos[0]-back_path[j][0])*1.0))    
                    sleep(1) 

                if back_path[j][1]>now_pos[1]: 
                    print('case2_1')
                    if drone_yaw ==180:
                        pass
                    elif drone_yaw ==0:
                        drone_yaw =180
                        tello.rotate_clockwise(180)
                        sleep(1)
                    tello.move_forward(int((back_path[j][1]-now_pos[1])*1.10))
                    sleep(1)
                    
                elif back_path[j][1]<now_pos[1]:
                    print('case2_2')
                    if drone_yaw ==0:
                        pass
                    elif drone_yaw == 180:
                        drone_yaw =0
                        tello.rotate_clockwise(180)
                        sleep(1)
                    tello.move_forward(int((now_pos[1]-back_path[j][1])*1.10))    
                    sleep(1)
                
                now_pos=back_path[j]
                print(j,now_pos) 
                j+=1
            else:
                print('case3')
                if back_path[j][1]>now_pos[1]: 
                    print('case3_1')
                    if drone_yaw ==180:
                        pass
                    elif drone_yaw ==0:
                        drone_yaw =180
                        tello.rotate_clockwise(180)
                        sleep(1)
                    tello.move_forward(int((back_path[j][1]-now_pos[1])*1.10))
                    sleep(1)
                    
                elif back_path[j][1]<now_pos[1]:
                    print('case3_2')
                    if drone_yaw ==0:
                        pass
                    elif drone_yaw == 180:
                        drone_yaw =0
                        tello.rotate_clockwise(180)
                        sleep(1)
                    tello.move_forward(int((now_pos[1]-back_path[j][1])*1.10))    
                    sleep(1)
                
                now_pos=back_path[j]
                print(j,now_pos)  
                j+=1
        else:
            break


keepRecording = False
keepMoveing = False
player.join()
