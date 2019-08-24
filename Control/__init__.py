import time
import numpy as np

import threading
import sys
import os

sys.path.append(os.path.dirname(__file__))

# For Test
# =====================================================================================================================
# 아래 값들을 바꿔서 실험해볼 수 있습니다.
CAR_SPEED = 150  # 차량의 기본 주행 속력 (km/h * 10)
KP = 1.2  # P gain
KD = 3  # D gain
DISTANCE_SQUARE = 12 # 타겟 점 선택 반경 (meter) 의 제곱
# =====================================================================================================================


class Control:
    def __init__(self, db):
        # class parameter

        self.db = db
        self.flag = db.flag
        self.__control_thread = threading.Thread(target=self.__main)

        # --------------------------------------------------------------------

        # 제어 알고리즘에 값을 넘겨주기 위한 packet
        # https://github.com/x2ever/HEVEN-AutonomousCar-2019/wiki/Required-Values-for-Control-Algorithms 참고

        # --------------------------------------------------------------------

        # PID control
        # PD 제어 기반
        # P 제어는 각도 오차에 비례하는 값을, D 제어는 현재 오차 각도와 이전 오차 각도의 차이값을 이용합니다.

        self.prev_angle_error = 0  # PD 제어를 위해 이전 각도 오차 저장이 필요.

        # --------------------------------------------------------------------

        # target list 를 저장해야 함.
        self.curr_target_list = []

        # --------------------------------------------------------------------

        # For parking mission
        self.parking_count = 0

    def __main(self):
        # =============================================================================================================
        # 경로 선택
        # meter 단위 target list

        # track for test
        total_track = self.db.path.gps_path_test_for_Control
        
        # 예선 track
        # total_track = self.db.path.gps_1_track

        # 본선 track
        # total_track = self.db.path.gps_2_track
        index = 0
        
        # UTM 경로를 사용하지 않을거라면, 현재 파티션 부분을 주석처리
        # =============================================================================================================

        '''# Initial target list
        self.curr_target_list = self.db.new_list.copy()'''

        # 자율주행 알고리즘 시작
        while not self.flag.system_stop:

            mission_num = 2

            # packet from Path Planning
            packet = [mission_num, 0, None, None, None]

            lidar_raw_data = self.db.lidar.data
            minimum_distance = lidar_raw_data[180] / 10
            min_theta = 180
            car_width = 180

            for theta in range(90, 270):
                if (minimum_distance > lidar_raw_data[theta] / 10) and \
                    ((lidar_raw_data[theta] / 10) * abs(np.cos(theta * np.pi / 360)) < (
                        car_width / 2)):  # 2 Conditions           ​
                    minimum_distance = lidar_raw_data[theta] / 10
                    min_theta = theta
                else:
                    pass
            distance_obs = minimum_distance * np.sin(min_theta * np.pi / 360)
            packet[2] = distance_obs

            # 현재 차체 좌표 (meter 단위)

            gps_data = self.db.gps.data
            #print(gps_data)
            minute_lad = gps_data[1] % 100
            degree_lad = (gps_data[1] - minute_lad)/100
            latitude = degree_lad + minute_lad/60

            minute_lon = gps_data[3] % 100
            degree_lon = (gps_data[3] - minute_lon)/100
            longitude = degree_lon + minute_lon/60
            
            p_curr = [latitude * 110000, longitude * 88800]

            # 현재 차체 방향 (정북 0도)

            car_angle = self.db.imu.data[2] + 180

            '''
            # 타겟 점 선택, 다음 리스트 요청 알고리즘

            if mission_num is 8:
                need parking path
            else:
                if index == SOME_NUMBER:     # 일정 인덱스가 되면
                    # 새로운 리스트와 이전에 저장되어 있던 리스트를 합치기
                    for point in self.db.new_list:
                        self.curr_target_list.append(point)

                elif index == LIST_SIZE:     # 마지막 index 에 도달하면
                    # 이전 target list 는 제거하고
                    self.curr_target_list = self.curr_target_list[LIST_SIZE:2*LIST_SIZE]
                    
                    # 다시 0번 인덱스부터 시작
                    index = 0

                else:
                    pass

            target_list = self.curr_target_list

            # target_list 준비 완료
            '''

            # 타겟 점 선택 알고리즘

            distance = (p_curr[0] - total_track[index][0]) ** 2 + (p_curr[1] - total_track[index][1]) ** 2

            '''
            if mission_num is 8:
                distance_square = 3
            else:
                pass
            '''

            # 주차 미션
            if mission_num is 8:
                if self.parking_count < 70000:  # 주차 정차 이전
                    if index == len(total_track) - 1:
                        pass
                    else:
                        if distance < DISTANCE_SQUARE:
                            index += 1
                        else:
                            pass

                else:  # 주차 정차 이후
                    if (p_curr[0] - total_track[index-1][0]) ** 2 + (p_curr[1] - total_track[index-1][1]) ** 2 < DISTANCE_SQUARE:
                        index -= 1
                    else:
                        pass

            # 그 외의 미션            
            else:            
                if distance < DISTANCE_SQUARE:
                    index += 1

                else:
                    pass
            
            p_targ = total_track[index]
            
            if mission_num is 8:
                packet[2] = len(total_track)
                packet[3] = index

            # 타겟 점 선택 완료

            # *********************************************************************************************************
            # 제어 알고리즘에 필요한 데이터 수신 완료
            # *********************************************************************************************************
            
            # 조향각 제어

            # path tracking
            curr_angle_error = self.__find_angle_error(car_angle, p_curr, p_targ)  # 현재 각도 오차를 구합니다.

            # 기어 설정, 만약 주차 미션의 경우 뒤로 갈 때 각도 오차를 다시 구함
            final_gear = 0x00

            if mission_num is 8:
                if self.parking_count == 70000:
                    if 180 >= curr_angle_error >= 90:
                        curr_angle_error = 180 - curr_angle_error
                        final_gear = 0x02

                    elif -90 >= curr_angle_error >= -180:
                        curr_angle_error = - (180 + curr_angle_error)
                        final_gear = 0x02

                    else:
                        final_gear = 0x02

                else:
                    final_gear = 0x00

            # PID control 으로 조향각 (degree) 을 구합니다.
            steer_angle = self.__calculate_pid(curr_angle_error, self.prev_angle_error)

            # 조향각 결정
            # ---------------------------------------------------------------------------------------------------------
            # 주차 미션일 경우, 따로 조향각과 속력 및 브레이크 제어

            if mission_num is 8:
                speed = 70
                target_speed, steer_angle, final_brake = self.__parking_mission(packet, speed, steer_angle)

            # 그 외에는, 미션별 속력 및 브레이크 제어

            else:
                speed = CAR_SPEED
                target_speed, final_brake = self.__do_mission(packet, speed)

            # 속력 최종 결정
            # ---------------------------------------------------------------------------------------------------------
            # 값 조정 (시리얼 패킷으로 보낼 값들 조정)

            # 조향각에 71을 곱합니다. (serial 통신을 위해)
            steer_angle_to_packet = steer_angle * 71

            # 만약 조향각이 2000/71도 보다 크거나, -2000/71도 보다 작다면, 값을 조정하면서 목표 속력을 줄입니다.
            final_steer_angle, final_speed = self.__constraint(steer_angle_to_packet, target_speed)

            # 브레이크 값이 있다면, 속력을 0으로
            if final_brake > 0:
                final_speed = 0
            # 값 조정 완료
            # ---------------------------------------------------------------------------------------------------------
            # 통신부로 보내주기

            self.db.platform.send_data.speed = final_speed
            self.db.platform.send_data.steer = final_steer_angle
            self.db.platform.send_data.gear = final_gear
            self.db.platform.send_data.brake = final_brake

            # print(f"final_steer_angle {final_steer_angle}, index {index}, count {self.parking_count}")
            #print(f"final_brake {final_brake}, index {index}, distance_obs {distance_obs}")
            # *********************************************************************************************************
            # 플랫폼으로 통신 완료
            # *********************************************************************************************************
            
            # 현재 각도 오차를 이전 각도 오차로 저장.
            self.prev_angle_error = curr_angle_error

            # Save control data for monitoring
            self.db.control_data.p_curr = p_curr.copy()
            self.db.control_data.p_targ = p_targ.copy()
            self.db.control_data.car_angle = car_angle
            self.db.control_data.distance = distance
            self.db.control_data.final_speed = final_speed
            self.db.control_data.final_steer_angle = final_steer_angle
            self.db.control_data.final_gear = final_gear
            self.db.control_data.final_brake = final_brake

    def __transform(self, enu_angle):
        # East = 0 을 기준으로 하는 ENU angle 을 local angle 로 변환
        angle = enu_angle - 90

        if 0 > angle >= -270:
            angle += 360

        elif angle > 360:
            angle -= 360

        else:
            pass

        return angle
        
    def __find_angle_error(self, car_angle, position_curr, position_targ):  # 현재 차체의 각도, 현재 차의 위치, 선택한 타겟점의 위치
        targetdir_x = position_targ[1] - position_curr[1]  # 타겟 방향의 벡터 = 타겟 위치 - 현재 위치
        targetdir_y = position_targ[0] - position_curr[0]

        target_angle = np.arctan2(targetdir_y, targetdir_x) * 180 / np.pi

        # 각 변환
        target_angle = (450 - target_angle) % 360

        # 각도 오차 구하기
        angle_error = target_angle - car_angle          # target_angle 에서 car_angle 을 빼서 각도 오차를 구함
        while angle_error >= 180:
            angle_error -= 360                          # 각도 오차의 절댓값이 180보다 커지면 보정
        if angle_error <= -180:
            angle_error += 360
        # -180도 ~ 180도 사이의 angle_error 를 얻게 됨.

        print(f"car_angle {car_angle}, target_angle {target_angle}, angle_error {angle_error}")
        return angle_error

    def __calculate_pid(self, current_angle_error, prev_angle_error):
        k_p = KP
        k_d = KD

        angle_error = current_angle_error

        # p control
        proportional_term = k_p * angle_error

        # d control
        if prev_angle_error - angle_error >= 5 or prev_angle_error - angle_error <= - 5:
            derivative_term = 0  # 너무 차이가 크게 나는 값일 경우, 진동이 클것으로 판단되어 제외
        else:
            derivative_term = k_d * (angle_error - prev_angle_error)
        steer_angle = proportional_term + derivative_term

        return steer_angle

    def __constraint(self, steer_angle, speed):  # 조향각이 너무 클 경우, 2000/71도로 조정하고 속력 감소
        if steer_angle >= 2000:
            steer_angle = 2000

        elif steer_angle <= -2000:
            steer_angle = -2000

        else:
            if steer_angle >= 400:
                speed -= 50

            elif steer_angle <= -400:
                speed -= 50

            else:
                pass
            
        return steer_angle, speed

    def __do_mission(self, packet, target_speed):
        """:packet:
        packet 설명
        packet = [0, None, data1, data2, data3]
        [0] : 현재 모드 번호
        [1] : None
        [2] : data1
        [3] : data2
        [4] : data3

        :return:
        목표 속력과 브레이크 값을 return 합니다."""

        mode_num = packet[0]
        brake = 0

        if mode_num is 0 or None:
            if packet[4] is True:
                target_speed -= 50
            else:
                pass

        elif mode_num is 1:       # 정적 장애물
            target_speed -= 30

        elif mode_num is 2:       # 동적 장애물
            

            if packet[4] is True:
                target_speed -= 50

            # 동적 장애물 까지의 거리
            distance_obstacle = packet[2]

            if 500 <= distance_obstacle < 700:
                brake = 30

            elif 300 <= distance_obstacle < 500:
                brake = 50

            elif distance_obstacle < 300:
                brake = 70

        elif mode_num is 3:
            target_speed -= 30

        elif mode_num is 4:
            target_speed -= 30

        elif mode_num is 5:
            target_speed -= 30

        elif mode_num is 6:
            target_speed -= 30

            if packet[4] is True:
                target_speed -= 50

            if packet[3] is not None:  # 좌회전 신호가 보인다면
                if packet[3] is True:  # 좌회전 신호가 켜져있다면, 주행 계속
                    pass

                else:  # 좌회전 신호가 아니라면, 정지선 앞에 멈춰야 함
                    distance_stop = packet[2]  # 정지선까지의 거리

                    if distance_stop < 500:
                        brake = 30

                    elif distance_stop < 300:
                        brake = 60

                    elif distance_stop < 150:
                        brake = 110

            else:  # 좌회전 신호가 안보인다면, 그냥 주행 계속
                pass

        elif mode_num is 7:
            target_speed -= 30

            if packet[3] is not None:  # 적색 신호가 보인다면
                if packet[3] is True:  # 적색 신호가 켜져있다면, 정지선 앞에 멈춰야 함
                    distance_stop = packet[2]  # 정지선까지의 거리

                    if distance_stop < 500:
                        brake = 15

                    elif distance_stop < 300:
                        brake = 30

                    elif distance_stop < 150:
                        brake = 60

                else:  # 적색 신호가 아니라면, 주행 계속
                    pass

            else:  # 적색 신호가 안보인다면, 그냥 주행 계속
                pass

        return target_speed, brake

    def __parking_mission(self, packet, speed, steer):
        parking_track_length = packet[2]
        index = packet[3]

        final_speed = speed
        final_steer_angle = steer
        final_brake = 0

        if self.parking_count == 70000:
            pass

        elif index == parking_track_length - 1:
            if self.parking_count < 30000:
                final_steer_angle = 0
                final_speed = 30
                final_brake = 0
            elif 30000 <= self.parking_count < 70000:
                final_steer_angle = 0
                final_speed = 0
                final_brake = 70

            self.parking_count += 1

        else:
            pass

        return final_speed, final_steer_angle, final_brake
    # -------------------------------------------------------------------------------------------------
    # threading

    def start(self):
        print("Starting Control thread...")
        self.__control_thread.start()
        time.sleep(1)
        print("Control thread start!")

    def join(self):
        print("Terminating Control thread...")
        self.__control_thread.join()
        time.sleep(0.1)
        print("Control termination complete!")
