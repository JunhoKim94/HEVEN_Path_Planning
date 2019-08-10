'''
#################### PATH PLAN TEAM ####################

## ABOUT
- target point에 따른 path를 반환해주는 코드.

## INPUT & OUTPUT
- Input: Combine에서 받은 target, map
- Output: 제어에 넘길 Path

'''
from Combine import Combine
from hybrid_astar import hybrid_a_star

class Path_Planning:  # Mission으로부터 mission number를 받아 그에 맞는 제어에 넘겨줄 list를 반환해줌.
    def __init__(self, mission_number):  # 초기화
        combine = Combine(mission_number)
        self.__local_target = combine.target
        self.__map = combine.map
        self.__path = [(0,0)]

    def make_path(self):
        # Path 만들기 짜주세요ㅠ

    @property
    def path(self):
        return self.__path
     

