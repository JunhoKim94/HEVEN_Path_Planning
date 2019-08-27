'''
#################### PATH PLAN TEAM ####################

## ABOUT
- 각 미션 별로 제어팀에게 경로 정보가 담긴 packet을 넘겨줌

#https://github.com/x2ever/HEVEN-AutonomousCar-2019/wiki/Required-Values-for-Control-Algorithms
#제어팀 wiki 참조 

# +---+-------- packet[5] ----------+
# | 0 | mission number              |
# +---+-----------------------------+ 
# | 1 | coordinates of the targets  |
# +---+-----------------------------+
# | 2 | (depends on mission)        |
# +---+-----------------------------+
# | 3 | (depends on mission)        |
# +---+-----------------------------+
# | 4 | is school zone?             |
# +---+-----------------------------+


## INPUT & OUTPUT
- input: pathplan.py에서 표지판에 따른 YOLO 값 >> mission = Mission(1), mission.getpath()
- output: 각 미션 별 packet

'''

########### IMPORT MODULE ###########
from Lane_Detection import Lane_Detection
from YOLO import get_mission_number
from Database import Database
from Path_Planning import Path_Planning
import numpy as np
from line.util import make_binary
import Parking
#####################################

########## IMPORT INSTANCE ##########\
# yolo = yolo()
#####################################



class Mission:
    def __init__(self, mission_num):
        self.db = Database()
        self.lane = Lane_Detection()
        self.packet = [0, None, None, None, None]
        self.path_planning = Path_Planning(1,self.db)

    def make_packet(self):
        #각 미션 인덱스 별로 패킷 만들기
        if(self.db.now_idx < 10):
            self._path_tracking()
        elif(self.db.now_idx < 20):
            self._static_obstacle()
        elif(self.db.now_idx < 30):
            self._dynamic_obstacle()
        elif(self.db.now_idx < 40):
            self._non_signal_left()
        elif(self.db.now_idx < 50):
            self._non_signal_right()
        elif(self.db.now_idx < 60):
            self._signal_left()
        elif(self.db.now_idx < 70):
            self._signal_straight()
        elif(self.db.now_idx < 20):
            self._parking()

    ############### MISSON ###############

    ## 0. 직선 주행(기본 주행)
    def _path_tracking(self):
        self.packet = [0, None, None, None, None]
        #True일 경우 빨강
        self.packet[4] = self.lane.get_floor_color(self.db.main_cam.data)
        self.db.mission_packet = self.packet

    ## 1. 정적 장애물 미션
    def _static_obstacle(self):
        self.packet = [1, None, None, None, None]
        #경로 생성하기
        self.path_planning.make_path()
        self.packet[1] = self.path_planning.path()
        self.db.mission_packet = self.packet
        
    ## 2. 동적 장애물 미션
    def _dynamic_obstacle(self):
        self.packet = [2, None, None, None, None]
        lidar_raw_data = self.db.lidar.data
        lidar_raw_data = lidar_raw_data[150:210]
        for i in lidar_raw_data:
            if(i > 100000):
                lidar_raw_data.pop(i)
        distance = np.mean(lidar_raw_data) / 1000
        self.packet[2] = distance
        self.packet[4] = self.lane.get_floor_color(self.db.main_cam.data)
        self.db.mission_packet = self.packet
        
    ## 3. 비신호 직진 미션
    def _non_signal_straight(self):
        self.packet = [3, None, None, None, None]
        self.db.mission_packet = self.packet

    ## 4. 비신호 좌회전 미션
    def _non_signal_left(self):
        self.packet = [4, None, None, None, None]
        self.db.mission_packet = self.packet

    ## 5. 비신호 우회전 미션
    def _non_signal_right(self):
        self.packet = [5, None, None, None, None]
        self.db.mission_packet = self.packet

    ## 6. 신호 좌회전 미션
    def _signal_left(self):
        self.packet[0] = 6
        temp_img = make_binary(self.db.main_cam.data)
        self.packet[2] = self.lane.get_stop_line(temp_img)  # 정지선인식
        mission_number = get_mission_number()
        if(mission_number == 6): # 좌회전신호 판단 >> 주행 가능: 1, 멈춤: 0
            self.packet[3] = 0
        elif(mission_number == 3):
            self.packet[3] = 1
        self.db.mission_packet = self.packet

    ## 7. 신호 직진 미션
    def _signal_straight(self):
        self.packet[0] = 7
        temp_img = make_binary(self.db.main_cam.data)
        self.packet[2] = self.lane.get_stop_line(temp_img)  # 정지선인식
        mission_number = get_mission_number()
        if(mission_number == 7): # 적색 신호 판단 >> 주행 가능: 1, 멈춤: 0
            self.packet[3] = 0
        elif(mission_number == 4):
            self.packet[3] = 1
        self.db.mission_packet = self.packet

    ## 8. 주차 미션
    def _parking(self):
        temp_img = self.db.main_cam.data
        self.packet[0]=8
        re1, re2, goal = Parking.parking(temp_img)
        if(goal == 0):
            self.packet[1] = None
        else:
            self.path_planning.make_path()
            self.packet[1] = self.path_planning.path()
        self.db.mission_packet = self.packet
        
    #####################################
