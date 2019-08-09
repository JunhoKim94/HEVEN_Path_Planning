'''
#################### PATH PLAN TEAM ####################

## ABOUT
- Lidar를 파싱해서 반환해주는 코드

## INPUT & OUTPUT
- input: DB팀에서 받은 lidar 파일
- output: parsing된 lidar 파일

'''

########## IMPORT MODULE ##########
import numpy as np
import socket
import cv2
import threading
import time
###################################


######### GLOBAL VARIABLE #########
HOST = '127.0.0.1'
PORT = 10018
BUFF = 57600

PADDING = 5 ## 나중에 차 크기 측정해서 패딩 결정!!
TRANSFORM = 15 ## 나중에 차 크기 측정해서 camera <-> lidar 변환값 결정
WIDTH = 480
HEIGHT = 320
###################################


class Lidar:
    def __init__(self):
        ########## open files ##########
        self._HOST = '127.0.0.1'
        self._PORT = 10018
        self._BUFF = 57600
        self._getData = 0 # 데이터를 받을때는 1, 안받을때는 0
        self.parsed_data = None
        self.points = None

        self.sock_lidar = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock_lidar.connect((HOST, PORT))
        ################################

        ######## lidar 부분 영상처리 ########
        raw = sock_lidar.recv(BUFF).decode()

        lidar_thread = threading.Thread(target=self.get_data)
        lidar_thread.start()


    ## 라이다 데이터를 받을 때
    def get_data(self):
        self._getData = 1
        while True:
            raw = self.sock_lidar.recv(BUFF).decode()
            if not raw.__contains__('sEA'):
                if cv2.waitKey(1) & 0xff == ord(' '): continue
                _parsed_data = raw.split(' ')[116:477] ## raw lidar file을 parsing해줌
                self.parsed_data = [int(item, 16) for item in _parsed_data]

            if self._getData = 0: break
        self.sock_lidar.close()


    ## 라이다 데이터를 그만 받을 때
    def stop_data(self):
        self._getData = 0



    ## parsing된 file에서 lidar 점을 추출해줌
    def making_point_lidar(self): ## data: 찍을 점(parsing된 lidar 파일)
        current_frame = np.zeros((HEIGHT, WIDTH), np.uint8)
        points = np.full((361, 2), -1000, np.int)  # 점 찍을 좌표들을 담을 어레이 (x, y), 멀리 -1000 으로 채워둠.

        data_list = [int(item, 16) for item in self.parsed_data]
        for theta in range(0, 361):
            r = data_list[theta] / TRANSFORM  # 차에서 장애물까지의 거리, 단위는 cm

            if 2 <= r:  # 라이다 바로 앞 1cm 의 노이즈는 무시

                # r-theta 를 x-y 로 바꿔서 (실제에서의 위치, 단위는 cm)
                x = r * np.cos(np.radians(0.5 * theta))
                y = r * np.sin(np.radians(0.5 * theta))
                # 좌표 변환, 화면에서 보이는 좌표(왼쪽 위가 (0, 0))에 맞춰서 집어넣는다
                points[theta][0] = round(x) + WIDTH / 2
                points[theta][1] = HEIGHT - round(y)

        self.points = points


    ## 추출된 점을 빈 화면에 출력함(lidar만)
    def draw_lidar(self):
        current_frame = np.zeros((HEIGHT, WIDTH), np.uint8)
        for point in self.points:  # 장애물들에 대하여
            cv2.circle(current_frame, tuple(point), PADDING, 255, -1)  # 캔버스에 점 찍기
        cv2.imshow("lidar", current_frame) ## lidar만 출력


    ## 추출된 점을 lane 위에 표시(lane + lidar)
    def draw_lidar_on_lane(self, img): ## img: 점을 찍을 순간적인 lane frame
        for point in self.points:  # 장애물들에 대하여
            cv2.circle(img, tuple(point), PADDING, 255, -1)  # 캔버스에 점 찍기
        cv2.imshow("lane + lidar", img) ## lane + lidar
        return img

if __name__ == "__main__":
    current_lidar = Lidar()
    current_lidar.get_data()