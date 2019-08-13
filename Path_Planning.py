'''
#################### PATH PLAN TEAM ####################

## ABOUT
- target point에 따른 path를 반환해주는 코드.

## INPUT & OUTPUT
- Input: Combine에서 받은 target, map
- Output: 제어에 넘길 Path

'''
from Combine import Combine
from hybrid_astar.hybrid_a_star import HybridAStar
from hybrid_astar.car import Car
from hybrid_astar.mapinfo import MapInfo

import Database

class Path_Planning:  # Mission으로부터 mission number를 받아 그에 맞는 제어에 넘겨줄 list를 반환해줌.
    def __init__(self, mission_number):  # 초기화
        self.db = Database()
        self.combine = Combine(mission_number,self.db)
        self.__local_target = combine.target
        self.__map = combine.map
        self.__path = [(0,0,0)]

    def make_path(self):
            
            m = MapInfo(640, 480)
            vehicle = Car(10.0, 5.0)
            start = (10, 20, 0) # self.db.gps.data + self.db.imu.data
            end = (35, 5, 0) # self.db.gps.data +self.db.imu.data
            m.start = start
            m.end = end
            ob = self.__map[self.__map > 0]
            m.obstacle = ob
            vehicle.set_position(m.start)
            vehicle.show()
            m.update()
            
            plan = HybridAStar(m.start, m.end, m, vehicle, r=5.0)
            plan.run(False)
            xs,ys,yaws = plan.reconstruct_path()
            path = []
            
            for cord in zip(xs,ys,yaws):
                path.append(cord)
            
            self.__path = path

    @property
    def path(self):
        return self.__path
     
if __name__ == "__main__":
    Path = Path_Planning(0)
    Path.make_path()
    p = Path.path
