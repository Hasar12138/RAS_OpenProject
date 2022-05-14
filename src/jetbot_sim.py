import numpy as np
import cv2
import time
import math

# robot = Robot()

def aruco_detect(image):
    frame = image
    # frame=cv2.resize(frame,[700,700],interpolation=cv2.INTER_CUBIC)
    #灰度话
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #设置预定义的字典
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)
    #使用默认值初始化检测器参数
    parameters =  cv2.aruco.DetectorParameters_create()
    #使用aruco.detectMarkers()函数可以检测到marker，返回ID和标志板的4个角点坐标
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray,aruco_dict,parameters=parameters)
    frame_x = int(frame.shape[1]/2)
    frame_y = int(frame.shape[0]/2)
    if ids is not None:
  
        center_arcuo = get_corner_center(corners)
        arcuo_x,arcuo_y = center_arcuo[0]
        cv2.circle(frame, (arcuo_x,arcuo_y), 8, (0, 0, 255), 3, 8, 0)
        cv2.circle(frame, (frame_x,frame_y), 10, (255, 255, 0), 3, 8, 0)
        cv2.line(frame, (frame_x,frame_y), (arcuo_x,arcuo_y), (0, 255, 0), 2, 4)
        # cv2.imshow('image',frame)
        # cv2.waitKey(0)
    
        # cv2.circle(frame)
        print('Have found')
        return ids,arcuo_x,arcuo_y,frame_x,frame_y
    else:
        print('not find arcuo in the frame')
        return ids,None,None,frame_x,frame_y
def get_corner_center(corners):
    center = []
    for i in range(len(corners)):
        x = int(np.sum(corners[i][0].T[0])/4.0)
        y = int(np.sum(corners[i][0].T[1])/4.0)
        center.append([x,y])
    return center

    
def motion_decision(arcuo_x,arcuo_y,frame_x,frame_y):
    Manhattan_distance = abs(arcuo_x - frame_x) + abs(arcuo_y - frame_y) # (600,600) Manhattan_distance max =1200
    Euclidean_distance = int(math.sqrt((arcuo_x - frame_x)**2 + (arcuo_y - frame_y)**2))
    distance_threshold = 100
    delta_x_threshold = 50
    delta_x = abs(arcuo_x - frame_x)
    delta_y = abs(arcuo_y - frame_y)
    if Euclidean_distance > distance_threshold:
        if delta_x <= delta_x_threshold: #不需要转弯 need to turn the direction
            if arcuo_y>=int(frame_y):#前进
                left_speed = 0.3
                right_speed = 0.3
                print('forward')
            else: 
                left_speed = -0.3
                right_speed = -0.3
                print('backward')
            
        elif delta_x >= delta_x_threshold: #需转弯
            if arcuo_x<int(frame_x) and arcuo_y>=int(frame_y): #region 1
                left_speed = 0.25
                right_speed = 0.35
                print('region 1')
            elif arcuo_x>int(frame_x) and arcuo_y>=int(frame_y): #region 2
                left_speed = 0.35
                right_speed = 0.25
                print('region 2')
            elif arcuo_x<int(frame_x) and arcuo_y<=int(frame_y): #region 3
                left_speed = -0.25
                right_speed = -0.35
                print('region 3')
            elif arcuo_x>int(frame_x) and arcuo_y<=int(frame_y): #region 4
                left_speed = -0.35
                right_speed = -0.25  
                print('region 4')  
        else:
            left_speed = 0
            right_speed = 0
    #The drone is on the top of the photo-> need to backwork
    ############################### ->x
    #           delta_x           #
    #            ------           #
    #         3#       #4         #
    #         #         #         #
    #                             #                             
    #         #         #         #
    #         1#       #2         #
    #                             #
    #                             #
    #                             #
    ###############################
    #y
     #The drone is at the bottom of the photo-> need to forward
    else: # STOP!
        left_speed = 0
        right_speed = 0
   

    #############    control_motor     ########################
    # robot.left_motor.value = left_speed
    # robot.right_motor.value = right_speed
    # time.sleep(0.2)
    ################################################
    # print(Manhattan_distance)


if __name__ == '__main__':
    # need while(true)
    # image = cv2.imread('/home/yunfeng/Documents/RAS/pj/test1.png')
    capture = cv2.VideoCapture(0)
    while(True):
        ref, image = capture.read()
        # image = cv2.flip(image,1) 
        ids,arcuo_x,arcuo_y,frame_x,frame_y =aruco_detect(image)
        if ids is not None:
            motion_decision(arcuo_x,arcuo_y,frame_x,frame_y)
    # cv2.circle(image, (arcuo_x,arcuo_y), 10, (255, 255, 0), 3, 8, 0)
        # image = cv2.flip(image,1) 
        cv2.imshow('image',image)
        cv2.waitKey(1)
    print('=======')