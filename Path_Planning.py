'''
#################### PATH PLAN TEAM ####################

## ABOUT
- target point에 따른 path를 반환해주는 코드.

## INPUT & OUTPUT
- input: 미션 번호 & (combine으로부터 loacl target점과 map을 입력받음)
- output: 제어에 넘겨줄 path의 list

'''

from Combine import Combine
from hybrid_astar import hybrid_a_star

class Path_Planning:  # Mission으로부터 mission number를 받아 그에 맞는 제어에 넘겨줄 list를 반환해줌.
    def __init__(self, mission_number):  # 초기화
        combine = Combine(mission_number)
        _local_target = combine.get_target()
        _map = combine.get_map()

    def get_path(self):  # 제어팀에게 넘겨줄 path point list를 생성하는 코드
        path = [(0,0)]
        #  _local_target과 _map을 사용하여 path를 생성함
        return path