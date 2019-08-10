'''
#################### PATH PLAN TEAM ####################

## ABOUT
- 미션 번호에 따라 lane과 lidar를 적절히 혼합하여 target, map을 반환해주는 코드

## INPUT & OUTPUT
- Input: Mission number, DB
- Output: Path_Planning에 넘겨줄 target, map

'''
import Lane_Detection
import Lidar
import cv2
import numpy as np

# Input : DB, Mission number 
# Output : target, map
class Combine:  # 나중에 이 함수에 display 메소드도 추가해야 할듯..?
    def __init__(self, mission_number,database):  # 초기화
        self.__mission_number = mission_number
        self.database = database
        self.radius = 5 # 5m to see
        self.map = [(0,0)]
        self.target = (0,0)
        
        if self.__mission_num == 0: self.__static_obstacle() 
        elif self.__mission_num == 1: self.__dynamic_obstacle()
        elif self.__mission_num == 2: self.__parking() 

    def update_map(self):
        img = self.database.main_cam.data
        Lane_map = Lane_Detection(img)
        lidar = self.database.lidar.data
        
        x = np.sin(angle)*lidar
        y = np.cos(angle)*lidar

        position = np.vstack([x,y])
        
        '''
        calibration 필요 --> pixel 하나당 몇 m 인지
        lm per 480 pixel
        '''
        l = 1
        x= 480 *x / l
        y = 680 *y / l /2
        
        for i in range(len(lidar)):
            cv2.line(img,(y,x),(y,x),(255,255,255))
        self.map = img

    ########## 각 상황에 맞게 Lidar, Lane_Detection 이용하여 함수 짜기 ##########
    ## 신호/비신호는 path를 짜는것에 있어서는 같을 것 같아 하나로 묶음
    ## 각 상황에 맞는 map과 local target값을 넣으면
    def __static_obstacle(self):
        # map, target 만들기
        
    def __dynamic_obstacle(self):
        # map, target 만들기
        
    def __parking(self):
        # map, target 만들기
