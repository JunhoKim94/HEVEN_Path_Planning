import cv2
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import skewnorm
from line.util import make_binary, smoothing, draw_points

# 
'''
#################### PATH PLAN TEAM ####################
## ABOUT
- 차선을 인식해서 왼쪽차선, 오른쪽차선, 정지선의 위치를 반환해주는 코드
## INPUT & OUTPUT
- input: 카메라 원
- output: 각 미션 별 packet
'''

#다항식 차수
poly_order = 2
#조사창 너비 정하기 (이미지 좌우 크기/50)
n_windows = 20

windows_width = 20
#최대 픽셀 수
max_pixel_num = 100
#최소 픽셀 수
min_pixel_num = 30

display = (800, 600)

class Line:
    def __init__(self,current_line = None,curve = 0,detect = False):
        self.current_fit = [np.array([False])]
        self.prevx = []
        self.allx = None
        self.ally = None
        self.curve = curve
        self.detect = detect
        # polynomial coefficients for the most recent fit


## Lane_Detection.py
class Lane_Detection: #Lane_Detction 클래스 생성후, original img 변경
    def __init__(self, img,left,right):  # 초기화
        self.original_img=img
        
        self.left = left
        self.right = right
        
        #midpoint = np.int(display[1]/2)

        
    def run(self):
        self.binary_img = make_binary(self.original_img)
        cv2.imshow('bin', self.binary_img)
        self.map = self.search_lines(self.binary_img)
        
    def get_stop_line(self,img):  # 정지선을 반환하는 코드(정지선 제일 앞 부분)
        cv2.imshow('hi', img)
        left_high = (int(0.3*display[0]), int(0.94*display[1]))
        right_low = (int(0.7*display[0]), int(0.98*display[1]))
        img1 = img[left_high[1]:right_low[1], left_high[0]:right_low[0]]
        img1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
        num = cv2.countNonZero(img1)
        cv2.rectangle(img, left_high, right_low, (0, 255, 0), 2)
        Area=(right_low[0] - left_high[0])*(right_low[1]-left_high[1])
        
        if(num > int(Area*0.4)):
            return True
        else:
            return False
        
    def get_floor_color(self, img):
        img1 = cv2.cvtColor(img, cv2.COLOR_HSV2BGR) #상황에 따라 적절하게 모드변환
        left_high = (int(0.4*display[0]), int(0.5*display[1]))
        right_low = (int(0.6*display[0]), int(0.9*display[1]))
        img2 = img1[left_high[1]:right_low[1], left_high[0]:right_low[0]]
        b, g, r = cv2.split(img2)
        avg = np.mean(r)
        
        if(avg>160):
            return('red')
            
    def search_lines(self,b_img):
        
        histogram = np.sum(b_img[int(b_img.shape[0]/1.5):, :], axis=0)

        monitor = np.dstack((b_img, b_img, b_img))
        
        midpoint = np.int(histogram.shape[0] / 2)
        
        left_sk = np.linspace(0.3, 1, 0.8*midpoint)
        left_sk = np.concatenate([left_sk,np.linspace(1, 0, 0.2*midpoint)])
        
        right_sk = np.linspace(0, 1, 0.2*midpoint)
        right_sk = np.concatenate([right_sk,np.linspace(1, 0.3, 0.8*midpoint)])
                    
        left_x_max = np.argmax(left_sk*histogram[:midpoint])
        right_x_max = np.argmax(right_sk*histogram[midpoint:]) + midpoint

        window_height = np.int(b_img.shape[0]/n_windows)
        
        #print(b_img.nonzero())
        
        current_left = left_x_max
        current_right = right_x_max
        
        left_lane_x = []
        right_lane_x = []
        left_lane_y = []
        right_lane_y = []
        
        for windows in range(n_windows):
            win_y_low = b_img.shape[0] - (windows+1) * window_height
            win_y_high = win_y_low +window_height
            left_x_low = current_left - windows_width
            left_x_high = current_left + windows_width
            right_x_low = current_right - windows_width
            right_x_high = current_right + windows_width
            
            
            cv2.rectangle(monitor, (left_x_low, win_y_low), (left_x_high, win_y_high), (0, 255, 0), 2)
            cv2.rectangle(monitor, (right_x_low, win_y_low), (right_x_high, win_y_high), (0, 255, 255), 2)
            
            left_x = np.array(b_img[win_y_low:win_y_high,left_x_low:left_x_high].nonzero()[1]) + left_x_low
            left_y = np.array(b_img[win_y_low:win_y_high,left_x_low:left_x_high].nonzero()[0]) + win_y_low
            right_x = b_img[win_y_low:win_y_high,right_x_low:right_x_high].nonzero()[1] + right_x_low
            right_y = b_img[win_y_low:win_y_high,right_x_low:right_x_high].nonzero()[0] + win_y_low

            # If you found > minpix pixels, recenter next window on their mean position
            if len(left_x) > min_pixel_num:
                current_left = np.int(np.mean(left_x))
            if len(right_x) > min_pixel_num:
                current_right = np.int(np.mean(right_x))
                
            left_lane_x.append(left_x)
            right_lane_x.append(right_x)
            left_lane_y.append(left_y)
            right_lane_y.append(right_y)
        
        
        lx = np.concatenate(left_lane_x)
        ly = np.concatenate(left_lane_y)
        rx = np.concatenate(right_lane_x)
        ry = np.concatenate(right_lane_y)

        self.left.current_line = [lx,ly]
        self.right.current_line = [rx,ry]
        
        ploty = np.linspace(0, b_img.shape[0] - 1, b_img.shape[0])
        
        left_fit = np.polyfit(ly, lx, 2)
        right_fit = np.polyfit(ry, rx, 2)
            
        line_left = np.poly1d(left_fit)
        line_right = np.poly1d(right_fit)
                        
        y1 = line_left(ploty)
        y2 = line_right(ploty)
               
        if (len(rx)>2000) & (len(lx) >2000):
            self.left.prevx.append(y1)
            self.right.prevx.append(y2)
    
            self.left.detect = True
            self.right.detect = True
        else:
            self.left.detect = False
            self.right.detect = False
        
        num = 5
        
        if len(self.left.prevx) > num:
            self.left.prevx.pop(0)
            left_avg_line = smoothing(self.left.prevx, num)
            left_avg_fit = np.polyfit(ploty, left_avg_line, poly_order)
            l = np.poly1d(left_avg_fit)
            left_fit_plotx = l(ploty)
            self.left.current_fit = left_avg_fit
            self.left.allx, self.left.ally = left_fit_plotx, ploty
        else:
            
            self.left.current_fit = left_fit
            self.left.allx, self.left.ally = y1, ploty
    
        if len(self.right.prevx) > num:
            self.right.prevx.pop(0)
            right_avg_line = smoothing(self.right.prevx, num)
            right_avg_fit = np.polyfit(ploty, right_avg_line, poly_order)
            r = np.poly1d(right_avg_fit)
            right_fit_plotx = r(ploty)
            self.right.current_fit = right_avg_fit
            self.right.allx, self.right.ally = right_fit_plotx, ploty
            
        else:
            self.right.current_fit = right_fit
            self.right.allx, self.right.ally = y2, ploty
        
        draw_points(monitor,self.left.allx,ploty,(0,0,255),3)
        draw_points(monitor,self.right.allx,ploty,(0,255,0),3)

        cv2.imshow("ss",monitor)
        return monitor


            


def show_video():
    video="./video/pre_lane_Trim.mp4"
    cap = cv2.VideoCapture(video)
    
    left = Line()
    right = Line()
    
    while True:
        ret, img = cap.read()
        if not ret:
            print('비디오 끝')
            break
        lane = Lane_Detection(img, left, right)
        lane.run()
        cv2.imshow("zzz",img)
        
        if cv2.waitKey(1) & 0xFF == 27:
            break        

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    show_video()

