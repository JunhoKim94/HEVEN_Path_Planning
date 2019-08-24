'''
#################### PATH PLAN TEAM ####################

## ABOUT
- 미션 번호에 따라 lane과 lidar를 적절히 혼합하여 target, map을 반환해주는 코드

## INPUT & OUTPUT
- Input: Mission number, DB
- Output: Path_Planning에 넘겨줄 target, map
#y 20 pixel  =1m  x 60pixel = 1m
'''
import sys
import os
sys.path.append(os.path.dirname(__file__))

from Lane_Detection import Lane_Detection
import cv2
import numpy as np
import math
from Database import Database
from Basic import BasicPath


import matplotlib.pyplot as plt
#1m/pixel
x_scale = 20
y_scale = 20
# Input : DB, Mission number 
# Output : target, map
class Combine:  # 나중에 이 함수에 display 메소드도 추가해야 할듯..?
    def __init__(self, mission_number,database):  # 초기화
        self.__mission_number = mission_number
        self.database = database
        self.Lane_detect = Lane_Detection(poly_order=1)
        self.radius = 5 # 5m to see
        self.map = [(0,0)]
        self.target = (0,0)
        '''
        if self.__mission_num == 0: self.__static_obstacle() 
        elif self.__mission_num == 1: self.__dynamic_obstacle()
        elif self.__mission_num == 2: self.__parking() 
        '''
    def update_map(self):
        img = cv2.imread("./video/pre_lane_Moment.jpg")
        self.Lane_detect.run(img)#self.database.main_cam.data
        left_x = (self.Lane_detect.left.allx-200)/37 * 20
        right_x = (self.Lane_detect.right.allx-200)/37 * 20

        #대칭이동 (y= 400 에서)
        left_y = (800 - self.Lane_detect.left.ally)/35 * 20
        right_y = (800 - self.Lane_detect.right.ally)/35 * 20

        left = np.polyfit(left_y,left_x,2)
        right = np.polyfit(right_y,right_x,2)

        line_left = np.poly1d(left)
        line_right = np.poly1d(right)

        ob = [(int(line_left(i)), i) for i in range(800)] +[(int(line_right(i)),i) for i in range(800)]
        '''
        plt.plot(line_left(line),line)
        plt.plot(line_right(line),line)
        plt.show()
        '''
        lidar = [5645, 5642, 5635, 5643, 5622, 5630, 5643, 5642, 5640, 5658, 5634, 5634, 5657, 5671, 5660, 5667, 5675, 5687, 5703, 5697, 5688, 5702, 5713, 5724, 5730, 5743, 5764, 5775, 5789, 5778, 5797, 5804, 5826, 5835, 5851, 5875, 5871, 5876, 5892, 5907, 5942, 5959, 5976, 6002, 6019, 6025, 6043, 6073, 6098, 6121, 6139, 6184, 6186, 6220, 6237, 6274, 6296, 6324, 6341, 6378, 6409, 6445, 6466, 6507, 6533, 6565, 6594, 6646, 6676, 6709, 6763, 6794, 6831, 6887, 6927, 6959, 7011, 7046, 7107, 7166, 7202, 7261, 7310, 7349, 7422, 7478, 7454, 7378, 7398, 7446, 7515, 7575, 7778, 8001, 8083, 8145, 8232, 8292, 8392, 8472, 8534, 8624, 8725, 8807, 8894, 9004, 9095, 9218, 9312, 9412, 9519, 9634, 9745, 9883, 10005, 10156, 10275, 10429, 10558, 10699, 10860, 11029, 11153, 11332, 11502, 11653, 11833, 12049, 12234, 9900, 9862, 9827, 9772, 9746, 9704, 9683, 9648, 9622, 1000000, 1000000, 12750, 10612, 12203, 11673, 12208, 12306, 12352, 15484, 15436, 15555, 15676, 15754, 15994, 16313, 16923, 22700, 22645, 22133, 22106, 7735, 7756, 7736, 7694, 7708, 7719, 7686, 7681, 7697, 1000000, 1000000, 39561, 39528, 39522, 1000000, 1000000, 1000000, 1000000, 1000000, 1000000, 1000000, 1000000, 1000000, 1000000, 1000000, 1000000, 1000000, 1000000, 1000000, 1000000, 1000000, 1000000, 1000000, 1000000, 1000000, 1000000, 1000000, 1000000, 1000000, 1000000, 1000000, 1000000, 16534, 15390, 15021, 15051, 15099, 15128, 15044, 14681, 14640, 14609, 16723, 1000000, 1000000, 1000000, 1000000, 18237, 3247, 3235, 3237, 3247, 3312, 3287, 3245, 3223, 3212, 3204, 3236, 3253, 3289, 3334, 3321, 3332, 3326, 3377, 7194, 1000000, 13527, 13484, 13683, 13713, 13737, 13577, 13375, 13195, 13000, 12830, 12649, 12481, 12311, 12156, 12008, 8971, 8847, 5590, 5561, 5520, 5497, 5385, 5714, 5432, 5255, 5296, 5396, 5252, 5012, 5142, 5236, 5393, 4768, 4810, 5495, 5859, 4873, 4681, 5072, 4821, 5117, 4752, 4750, 4330, 4457, 4478, 4672, 4675, 4522, 4492, 4671, 4544, 4440, 4293, 4322, 4510, 4305, 4577, 4537, 4530, 4396, 4532, 4546, 4461, 4329, 4373, 4396, 4127, 3638, 4063, 4042, 3931, 3986, 4276, 4507, 4518, 4485, 4566, 4406, 4576, 4188, 4348, 4148, 4254, 4479, 4492, 3999, 3794, 3734, 3531, 3504, 4345, 4437, 4154, 3702, 3672, 3665, 3670, 3828, 3474, 3496, 4086, 3265, 3263, 3510, 3570, 3441, 3276, 3642, 3990, 4095, 4082, 4083, 4044, 3508, 3281, 3291, 3783, 3334, 3380, 3589, 3472, 3442, 3234]#self.database.lidar.data
        lidar = np.array(lidar)
        lidar[lidar > 15000] = 0
        print(lidar)
        angle = (np.linspace(0,180,361)/180 * np.pi)
        x = x_scale * np.cos(angle) * lidar/1000 + 100
        y = y_scale * np.sin(angle) * lidar/1000
        plt.plot(x,y)
        plt.show()

        for z in zip(x,y):
            ob.append(z)
        
        
        return ob
        
        
        '''
        calibration 필요 --> pixel 하나당 몇 m 인지
        lm per 480 pixel
        '''

    ########## 각 상황에 맞게 Lidar, Lane_Detection 이용하여 함수 짜기 ##########
    ## 신호/비신호는 path를 짜는것에 있어서는 같을 것 같아 하나로 묶음
    ## 각 상황에 맞는 map과 local target값을 넣으면
    def __static_obstacle(self):
        print(0)
        # map, target 만들기
        
    def __dynamic_obstacle(self):
        print(0)
        # map, target 만들기
        
    def __parking(self):
        '''
        Mission: 차량이 있는 주차공간과 장애인 전용 주차구간을 피해 주차를 함

        <Flow Chart>
        Lidar의 135도 방향중 일정 거리 이내에 ┌ 물체가 포착됨(이미 주차되어있는 차량이 있는 경우) -> return 0
                                          └ 포착되지 않음 -> 주차용 카메라에서 사각형을 찾음 ┌ 사각형이 검출되지 않는 경우 -> return 0
                                                                                       └ 사각형이 검출되는 경우 ┌ 장애인 주차구역인 경우 -> return 0
                                                                                                             └ 장애인 주차구역이 아닌경우 -> target, map을 저장, return 1
        '''
        
        '''
        # ==================== lidar에 장애물이 검출되나 확인 ====================== #
        database = Database(0, 0, 1, 1, 0)
        database.start()
        lidar_array = database.lidar
        if lidar_array[135] < LiDAR_MIN:  # lidar data의 135도에 물체가 있을 경우 그냥 넘어감
            self.map = None
            self.target = None
            return 0
        # ================================================================== #
        '''
            
        video = cv2.VideoCapture('./video/real_park1.mp4')
        while(video.isOpened()):
            _, cap = video.read()
            cv2.waitKey(10)
            cap = cv2.cvtColor(cap, cv2.COLOR_BGR2GRAY)
            cv2.imshow('original', cap)

            # cap = cv2.resize(cap, dsize = (0, 0), fx = 0.5, fy = 0.5, interpolation=cv2.INTER_LINEAR)  # 가로세로 크기 조절
            cap = cv2.resize(cap, dsize=(640, 480), interpolation=cv2.INTER_AREA)
            height = cap.shape[0]
            width = cap.shape[1]


            # ============================ 사각형 찾기 ============================== #
            ## bird view로 바꿈
            source_points = np.array([(0.4*width, 0.15*height), (0.1*width, 0.45*height), (0.8*width, 0.55*height), (0.75*width, 0.17*height)], np.float32) # 왼쪽 위부터 반시계방향
            destination_points = np.array([(0.3*width, 0), (0.3*width, height), (0.8*width, height), (0.8*width, 0)], np.float32) # 왼쪽 위부터 반시계방향
            warp_matrix = cv2.getPerspectiveTransform(source_points, destination_points)
            birdview_img = cv2.warpPerspective(cap, warp_matrix, (width, height), flags=cv2.INTER_LINEAR)

            cv2.imshow('why', birdview_img)


            ## 차선만 뽑아냄(흑백으로 바꿈)
            _, threshold_img = cv2.threshold(birdview_img, 180, 255, cv2.THRESH_BINARY)

            ## FOR DEBUGGING ##
            # 디버깅시 차선의 흑백 구분이 제대로 되나 확인
            # cv2.imshow('threshold_img', threshold_img)
            cv2.imshow('threshold_img', threshold_img)

            ## 사각형이 검출되지 않는 경우 -> 그냥 진행
            contours, _ = cv2.findContours(threshold_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            is_rectangle = 0  # 사각형이 있는 경우 1, 없는 경우 0
            rectangle_coordinate = [(0, 0), (0, 0), (0, 0), (0, 0)]  # 사각형이 있는 경우 꼭지점의 네 좌표를 저장
            center = [(0, 0)]

            for cnt in contours:
                #approx = cv2.approxPolyDP(cnt, 0.01 * cv2.arcLength(cnt, True), True)
                approx = cv2.approxPolyDP(cnt, 20, True)
                cv2.drawContours(birdview_img, [approx], 0, (0), 5)

                if len(approx) == 4:
                    is_rectangle = 1
                    ## 무게중심 구하기
                    raw_center = cv2.moments(approx)
                    center = (int(raw_center['m10']/raw_center['m00']), int(raw_center['m01']/raw_center['m00']))
                    ## 좌표 찾아 집어넣기(rectangle_coordinate에 네개의 꼭짓점을 각각 집어넣음)
                    rectangle_coordinate[0] = (approx.ravel()[0], approx.ravel()[1])
                    rectangle_coordinate[1] = (approx.ravel()[2], approx.ravel()[3])
                    rectangle_coordinate[2] = (approx.ravel()[4], approx.ravel()[5])
                    rectangle_coordinate[3] = (approx.ravel()[6], approx.ravel()[7])
                    break

            # ================================================================== #

            ## FOR DEBUGGING ##
            # 디버깅시 사각형 제대로 찾아지나 확인
            new_img = cv2.circle(threshold_img, rectangle_coordinate[0], 20, (255, 255, 255), -1)
            new_img = cv2.circle(threshold_img, rectangle_coordinate[1], 20, (255, 255, 255), -1)
            new_img = cv2.circle(threshold_img, rectangle_coordinate[2], 20, (255, 255, 255), -1)
            new_img = cv2.circle(threshold_img, rectangle_coordinate[3], 20, (255, 255, 255), -1)
            cv2.imshow("detect_rectangle", new_img)

            # ======================== 사각형이 없는경우 ========================== # >> 사각형이 없는 경우는 그냥 넘어감
            ## 장애인 구역인지 판단
            if is_rectangle == 0:
                self.map = None
                self.target = None
                continue

            handicap_radius = int(math.sqrt((rectangle_coordinate[0][0] - center[0]) * (rectangle_coordinate[0][0] - center[0])
                                + ((rectangle_coordinate[0][1] - center[1]) * (rectangle_coordinate[0][1] - center[1]))) / 6)

            if 2 * handicap_radius < 50: is_rectangle = 0
            if is_rectangle == 0:
                self.map = None
                self.target = None
                continue
            # ================================================================== #


            # ======================== 사각형이 있는 경우 ========================= # >> 사각형이 있는 경우 장애인 주차구간인지 아닌지 판단
            count_white = 0
            count_all = handicap_radius * handicap_radius * 4
            for i in range (center[0] - handicap_radius, center[0] + handicap_radius, 1):
                for j in range (center[1] - handicap_radius, center[1] + handicap_radius, 1):
                    if threshold_img[j][i] == 255: count_white = count_white + 1

            ## 장애인 주차구간일때 >> 장애인 주차구간인 경우 그냥 넘어감

            if count_white / count_all > 0.1:  # 흰색 비율이 무게중심 주위 정사각형의 10% 이상 차지할때는 장애인표시 있는걸로 생각
                self.map = None
                self.target = None
                continue
            ## 장애인 주차구간이 아닐때 >> target과 map을 저장
            else:  # 흰색 비율이 무게중심 주위 정사각형의 10% 미만일떄는 장애인표시 없는걸로 생각
                ## 주차공간의 사각형 중 차에서 가장 가까운 두 꼭짓점을 찾음
                close_find = 0
                far_find = 0
                close_point = [0, 1]
                far_point = [2, 3]
                for i in range (0, 4, 1):
                    if rectangle_coordinate[i][1] > center[1]:
                        close_point[close_find] = i
                        close_find = close_find + 1
                    else:
                        far_point[far_find] = i
                        far_find = far_find + 1
                if (rectangle_coordinate[close_point[0]][0] - rectangle_coordinate[close_point[1]][0])\
                        * (rectangle_coordinate[far_point[0]][0] - rectangle_coordinate[far_point[1]][0]) < 0: # close_point[0]과 far_point[0]이 center를 기준으로 대각선에 있지 않도록 조절
                    temp = close_point[0]
                    close_point[0] = close_point[1]
                    close_point[1] = temp

                print(rectangle_coordinate)
                # 모두 0으로 되어 있는 빈 Canvas(검정색)
                img = np.zeros((height, width, 3), np.uint8)
                new_map = cv2.line(img, rectangle_coordinate[far_point[0]], rectangle_coordinate[far_point[1]], (255,255,255), 10)  # 선은 흰색을 따라 그림
                new_map = cv2.line(new_map, rectangle_coordinate[far_point[0]], rectangle_coordinate[close_point[0]], (255, 255, 255), 10)  # 선은 흰색을 따라 그림
                new_map = cv2.line(new_map, rectangle_coordinate[far_point[1]], rectangle_coordinate[close_point[1]], (255, 255, 255), 10)  # 선은 흰색을 따라 그림
                new_map = cv2.circle(new_map, center, 10, (255, 255, 255), -1)

                #self.map = new_map
                #self.target = center

                ## FOR DEBUGGING ##
                # map 출력 테스트
                cv2.imshow('final', new_map)

            # ================================================================== #

if __name__ == "__main__":
    db = Database()
    db.start()
    Combine(0,db)
    db.join()
