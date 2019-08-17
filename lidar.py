# -*- coding: utf-8 -*-
import numpy as np
import cv2
from hybrid_astar.hybrid_a_star import HybridAStar
from hybrid_astar.car import Car
from hybrid_astar.mapinfo import MapInfo
import matplotlib.pyplot as plt

#1m/pixel
x_scale = 3
y_scale = 4
#Map scale
disp = [80,60]

def draw_points(img, x_points, y_points, color, thickness):
    for i in range(len(x_points)):
        if(x_points[i]!=None):
            cv2.rectangle(img, (int(x_points[i]), int(y_points[i])), (int(x_points[i]), int(y_points[i])),
                     color, thickness=1)

lidar = np.loadtxt("./video/lidar.txt",delimiter = ',')
b = np.loadtxt("./video/gps.txt", delimiter = ',')
c = np.loadtxt("./video/yaw.txt", delimiter = ',')

#현재 위치를 (0,0) 으로 바꾸기 (gps좌표 --> 현재좌표)
b = b - b[0,:]
#degree --> rad
c = c / 180 * np.pi

#현재 좌표계의 theta 만큼 회전변환
b_rot_x = np.cos(c[0])*b[:,0] - np.sin(c[0])*b[:,1]
b_rot_y = np.sin(c[0])*b[:,0] + np.cos(c[0])*b[:,1]

#연산을 위한 expand --> 실시간으로 데이터 받을때는 필요없을수도 있음.
b_rot_x = np.expand_dims(b_rot_x,1)
b_rot_y = np.expand_dims(b_rot_y,1)

c = c - c[0]
c = np.expand_dims(c,1)

#virtual map
map1 = np.zeros(disp,dtype = np.uint8)
maps = np.dstack([map1,map1,map1])

#각 local 좌표계에서 lidar의 x,y 값 --> m 단위
angle = (np.linspace(0,180,361)/180 * np.pi).reshape(1,361)
x = -x_scale * np.cos(angle) * lidar/1000
y = -y_scale * np.sin(angle) * lidar/1000

#noise (튀는값 올려버리기)
x[x==0] = 1000
y[y == 0] = 1000

#local 좌표계를 global로 변환
x_t = np.cos(c)*x - np.sin(c)*y + 30 + b_rot_x * x_scale
y_t = np.sin(c)*x + np.cos(c)*y + 80 - b_rot_y * y_scale

#virtual lane
ob = [(25, i) for i in range(80)] +[(42,i) for i in range(80)] 
# 800,600 -> 80,60
m = MapInfo(disp[1],disp[0])
vehicle = Car(0.5,1)
#End 좌표 설정해주기
m.end = (x_scale * b_rot_x[-1,0]+30, y_scale * b_rot_y[-1,0], np.pi/2)

#continous search
for i in range(0,5972,10):
    #Lidar Data를 cv2 array에 표시
    draw_points(maps,x_t[i,:],y_t[i,:],(255,255,255),3)
    
    #생성된 Path중 일부 Point만 tracking 한 후 재 search
    if i % 200 == 0:
        m.start = (x_scale * b_rot_x[i,0]+30, y_scale * b_rot_y[i,0]+1, -c[i,0]+np.pi/2)

        #0 이아닌 cv2 이미지의 값의 인덱스를 가져옴
        num = np.where(maps[:,:,0]>0)
        for z in zip(num[1],disp[0] - num[0]):
            ob.append(z)

        m.obstacle = ob

        vehicle.set_position(m.start)
        plan = HybridAStar(m.start, m.end, m, vehicle, r=5.0)
        
        if plan.run(False):
            xs,ys,yaws = plan.reconstruct_path()
            m.path = list(zip(xs, ys))
            m.update()
            m.show()
            m.obstacle = ob
            
            plt.cla()
        else:
            print("No Path")
            
    z = cv2.resize(maps,(500,300))
    cv2.imshow("Path_Making",z)
    
    if cv2.waitKey(10) & 0xFF == 27:
        break
    
#just search first time
'''
draw_points(maps,x_t[0,:],y_t[0,:],(255,255,255),3)

num = np.where(maps[:,:,0]>0)
for z in zip(num[1],disp[0] - num[0]):
    ob.append(z)

m.obstacle = ob
m.start = (x_scale * b_rot_x[0,0]+30, y_scale * b_rot_y[0,0]+1, -c[0,0]+np.pi/2)
vehicle.set_position(m.start)
plan = HybridAStar(m.start, m.end, m, vehicle, r=5)
if plan.run(False):
    xs,ys,yaws = plan.reconstruct_path()
    m.path = list(zip(xs, ys))
    m.update()
    for i in range(len(xs)):
        if i != len(xs) - 1 and i % 10 != 0:
            continue
        plt.cla()
        m.show()
        #m.start = (10, 10, np.pi / 2)
        #m.end = (50, 30, np.pi / 2)
        m.obstacle = ob
        m.path = list(zip(xs, ys))
        vehicle.set_position([xs[i], ys[i], yaws[i]])
        vehicle.show()
        plt.pause(0.1)
else:
    print("fail")
m.wait_close()
'''
    #draw_points(maps,xs,ys,(0,255,255),1)


cv2.destroyAllWindows()