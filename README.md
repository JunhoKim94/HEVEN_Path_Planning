각 팀에 필요할 것 같은 메소드와 클래스들을 정리해 놓았습니다.

앞에 _이 붙은 경우 그 클래스 안에서만 사용하는 내부 함수이며, _이 붙지 않은 경우 다른 파일에서 사용할 것 같은 외부 함수입니다.

메소드/클래스가 추가되거나 변경사항이 있을 경우 수정해주시면 감사하겠습니다!


### Lidar.py
```
class Lidar:
    def __init__(self):  # 초기화
    def get_data(self):  # Lidar 데이터를 받기 시작함
    def stop_data(self):  # Lidar 데이터를 그만 받음
```



### Lane_Detection.py  # input은 이미지
```
class Lane_Detection:
    def __init__(self):  # 초기화
    def left_line(self):  # numpy로 넘겨줌(이미지로)
    def right_line(self):  # numpy로 넘겨줌(이미지로)
    def combine_line(self):  # 둘다 출력하는 함수
    def get_stop_line(self):  # 정지선을 반환하는 코드(정지선 제일 앞 부분)
```



### Combine.py
```
class Combine:  # 나중에 이 함수에 display 메소드도 추가해야 할듯..?
    def __init__(self, mission_number):  # 초기화
        _mission_number = mission_number
        map = [(0,0)]
        local_target = (0,0)
        if self._mission_num == 0: self._path_tracking()
        elif self._mission_num == 1: self._static_obstacle()
        elif self._mission_num == 2: self._dynamic_obstacle()
        elif self._mission_num == 3: self._cross_straight()
        elif self._mission_num == 4: self._cross_left()
        elif self._mission_num == 5: self._cross_right()
        elif self._mission_num == 6: self._cross_left()
        elif self._mission_num == 7: self._cross_straight()
        elif self._mission_num == 8: self._parking()

    def get_map(self): return self.map
    def get_local_target(self): return self.local_target

    ########## 각 상황에 맞게 Lidar, Lane_Detection 이용하여 함수 짜기 ##########
    ## 신호/비신호는 path를 짜는것에 있어서는 같을 것 같아 하나로 묶음
    ## 각 상황에 맞는 map과 local target값을 넣으면
    def _path_tracking(self):
    def _static_obstacle(self):
    def _dynamic_obstacle(self):
    def _cross_left(self):
    def _cross_right(self):
    def _cross_straight(self):
    def _parking(self):
```


### Path_Planning.py
```
class Path_Planning:  # Mission으로부터 mission number를 받아 그에 맞는 제어에 넘겨줄 list를 반환해줌.
    def __init__(self, mission_number):  # 초기화
        combine = Combine(mission_number)
        _local_target = combine.get_map()
        _map = combine.get_map()
    def get_path(self):  # 제어팀에게 넘겨줄 path point list를 생성하는 코드
        path = [(0,0)]
        #  _local_target과 _map을 사용하여 path를 생성함
        return path
```


### Mission.py
```
class Mission:
    def __init__(self, mission_num):
        self._mission_num = mission_num  # 기본값은 0
        self.packet = [self._mission_num, [(0,0)], None, None, 0]  # default
    def get_packet(self):

    ########## 각 상황에 맞게 packet에 값을 집어넣음 ##########
    def _path_tracking(self):
    def _static_obstacle(self):
    def _dynamic_obstacle(self):
    def _non_signal_straight(self):
    def _non_signal_left(self):
    def _non_signal_right(self):
    def _signal_left(self):
    def _signal_straight(self):
    def _parking(self):
```


### Yolo.py
```
class YOLO:
    def __init__(self):  # 초기화
    def get_mission_number(self):
    def is_school_zone(self):  # school zone이면 1, school zone이 아니면 0
    def traffic_light(self):  # red 0, green 1, curve 2
```
