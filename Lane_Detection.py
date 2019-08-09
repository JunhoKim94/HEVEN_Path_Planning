import cv2
import numpy as np
from sklearn import linear_model
import matplotlib.pyplot as plt
from threshold import gradient_combine, hls_combine, comb_result

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
max_pixel_num = 80
#최소 픽셀 수
min_pixel_num = 30
#회귀를 위한 최소 라인 포인트 수
min_points_num = 4
#색상 범위 HSV
boundaries = [
    (np.array([161, 155, 84], dtype="uint8"), np.array([179, 255, 255], dtype="uint8")), # red1
    (np.array([0, 100, 70], dtype="uint8"), np.array([20, 255, 255], dtype="uint8")), # red2
    (np.array([94, 80, 200], dtype="uint8"), np.array([126, 255, 255], dtype="uint8")), # blue
    (np.array([10, 50, 50], dtype="uint8"), np.array([25, 255, 255], dtype="uint8")), #yellow
    (np.array([0, 0, 240], dtype="uint8"), np.array([180, 25, 255], dtype="uint8")) # white
]
# 모니터링 창 크기
display = (640, 480)
# 클로징 마스크 크기
kernel = np.ones((7,7), np.uint8)

## Lane_Detection.py
class Lane_Detection: #Lane_Detction 클래스 생성후, original img 변경
    def __init__(self, img):  # 초기화
        self.original_img=img
        
        #좌우 레인 디폴트 값
        self.x1_default = int(0.33*display[1])
        self.x2_default = int(0.66*display[1])
        self.height, self.width = img.shape[:2]
        #roi설정을 위한 vertics, 위부터 차례대로 왼쪽 위, 왼쪽 아래, 오른쪽 아래, 오른쪽 위다.
        self.vertics = np.array([[(int(0.2*self.width), int(0.2*self.height)), 
                                  (int(-0.5*self.width), int(0.8*self.height)),
                                  (int(1.5*self.width), int(0.8*self.height)),
                                  (int(0.8*self.width), int(0.2*self.height))]])
    
    
        '''
        self.vertics = np.array([[(int(0.33*self.width), int(0.83*self.height)), 
                                  (int(0.16*self.width), int(self.height)),
                                  (int(0.84*self.width), int(self.height)),
                                  (int(0.67*self.width), int(0.83*self.height))]])    
        '''
    
        #perspective변환을 위한 pts 설정
        self.pts1 = np.float32([(0.38*self.width, 0.08*self.height), 
                                (0*self.width, 0.5*self.height),
                                (1*self.width, 0.5*self.height),
                                (0.58*self.width, 0.08*self.height)])
        self.temp1, self.temp2 = display[:2]
        self.pts2 = np.float32([(0.33*self.temp1, 0*self.temp2),
                                (0.33*self.temp1, 1*self.temp2),
                                (0.66*self.temp1, 1*self.temp2),
                                (0.66*self.temp1, 0*self.temp2)])
        '''
        self.pts2 = np.float32([(0.33*self.temp1, 0*self.temp2),
                                (0.33*self.temp1, 1*self.temp2),
                                (0.66*self.temp1, 1*self.temp2),
                                (0.66*self.temp1, 0*self.temp2)])
        '''
        self.binary_img = self.make_binary()
        
        self.bin_height, self.bin_width = self.binary_img.shape[:2]
        cv2.imshow('bin', self.binary_img)
        
        self.search_lines(self.binary_img)
        
    
    def draw_both(self, img ,right_points, left_points):
        img1 = np.zeros_like(img)
        img1=cv2.cvtColor(img1, cv2.COLOR_GRAY2BGR)
        self.draw_points(img1, left_points, self.y_points, [255, 0, 0], thickness = 3)
        img1 = self.draw_lane(img1, left_points, self.y_points, [255, 0, 255], 'left')
        self.draw_points(img1, right_points, self.y_points, [0, 255, 0], thickness =3)
        img1 = self.draw_lane(img1, right_points, self.y_points, [0, 255, 0], 'right')
        cv2.imshow("zz",img1)
        return img1
    
    
    def get_stop_line(self):  # 정지선을 반환하는 코드(정지선 제일 앞 부분)
        print(0)
        
        
    def search_lines(self,b_img):
        histogram = np.sum(b_img[int(b_img.shape[0] / 2):, :], axis=0)

        monitor = np.dstack((b_img, b_img, b_img))
        
        midpoint = np.int(histogram.shape[0] / 2)
        left_x_max = np.argmax(histogram[:midpoint])
        right_x_max = np.argmax(histogram[midpoint:]) + midpoint

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
        cv2.imshow("ss",monitor)
        ''' 
        for lane in [left_lane_x,right_lane_x,left_lane_y,right_lane_y]:
            lane = np.concatenate(lane)
            print(lane)
            
        left_fit = np.polyfit(left_lane_y, left_lane_x, 2)
        right_fit = np.polyfit(right_lane_y, right_lane_x, 2)
        
        line_left = np.poly1d(left_fit)
        line_right = np.poly1d(right_fit)
        
        ploty = np.linspace(0, b_img.shape[0] - 1, b_img.shape[0])
        
        y1 = line_left(ploty)
        y2 = line_right(ploty)
        self.draw_points(monitor,left_lane_x,y1,(0,0,255),3)
        self.draw_points(monitor,right_lane_x,y2,(0,255,0),3)
        
        ransac = linear_model.RANSACRegressor()
        ransac.fit(self.add_square_feature(line_y),line_x)
        y = np.round(ransac.predict(self.add_square_feature(x)))
        '''
        return monitor

    def draw_points(self, img, x_points, y_points, color, thickness):

        try:
            for i in range(len(x_points)):
                if(x_points[i]!=None):
                    cv2.line(img, (int(x_points[i]), int(y_points[i])), (int(x_points[i]), int(y_points[i])),
                             color, thickness=3)
        except:
            print("error2")
            

    # 아래부터는 유틸함수
    def make_binary(self): # 이진화 이미지를 만드는 함수
        img = self.reg_of_int(self.original_img)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        warped_img = self.warp_image(img)
        img1 = np.zeros_like(warped_img)
        img1 = cv2.cvtColor(img1,cv2.COLOR_BGR2GRAY)
        
        #img1 = self.Detect(warped_img)
        
        for index in ['b','y','w']: # 색깔별로 채널 추출
            img2 = self.detectcolor(warped_img, index)
            img1 = cv2.bitwise_or(img1, img2)
        
        #warped_img = self.warp_image(img1) 
        
        img1 = self.closeimage(img1)
        return img1
    
    def add_square_feature(self,X):
        X = np.array(X)
        X = X.reshape(-1,1)
        X = np.concatenate([(X**2).reshape(-1,1), X], axis=1)
        return X
    
    def reg_of_int(self, img): # 이미지에서 roi 잘라내기
        self.mask = np.zeros_like(img)
        cv2.fillPoly(self.mask, self.vertics, (255,255,255))
        #cv2.fillPoly(self.mask, self.vertics_erase, (0,0,0))
        self.mask = cv2.bitwise_and(img, self.mask)
        return self.mask
        
    def warp_image(self, img): # 이미지 원근 변환
        M = cv2.getPerspectiveTransform(self.pts1, self.pts2)
        warped_img = cv2.warpPerspective(img, M, display)
        cv2.imshow("warp",warped_img)
        return warped_img
    
    def Detect(self,img):
        th_sobelx, th_sobely, th_mag, th_dir = (35, 100), (30, 255), (30, 255), (0.7, 1.3)
        th_h, th_l, th_s = (10, 100), (0, 60), (85, 255)
        combined_gradient = gradient_combine(img, th_sobelx, th_sobely, th_mag, th_dir)
        combined_hls = hls_combine(img, th_h, th_l, th_s)
        combined_result = comb_result(combined_gradient, combined_hls)
        return combined_result
        
    def detectcolor(self, img, color): # color = b, r, w, y / 이미지 상에서 색을 찾아 리턴
        minRange, maxRange = 0, 0
        if color == "w":
            (minRange, maxRange) = boundaries[4]
            mask = cv2.inRange(img, minRange, maxRange)
        elif color == "y":
            (minRange, maxRange) = boundaries[3]
            mask = cv2.inRange(img, minRange, maxRange)
        elif color == "b":
            (minRange, maxRange) = boundaries[2]
            mask = cv2.inRange(img, minRange, maxRange)
        elif color == "r":
            (minRange, maxRange) = boundaries[0]
            mask = cv2.inRange(img, minRange, maxRange)
            (minRange, maxRange) = boundaries[1]
            mask = mask + cv2.inRange(img, minRange, maxRange)
        else:
            print("In Image_util.py DetectColor - Wrong color Argument")
        return mask
    
    def closeimage(self, img):
        return cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)

def show_video():
    video="./video/upper1_Trim.mp4"
    cap = cv2.VideoCapture(video)
    while True:
        ret, img = cap.read()
        if not ret:
            print('비디오 끝')
            break
        img1 = Lane_Detection(img)
        cv2.imshow("adfa",img1.mask)
        cv2.imshow("zzz",img)
        '''
        b,g,r = cv2.split(img)
        ret , r_th = cv2.threshold(r,220,255,cv2.THRESH_BINARY)
        img1 = r_th
        '''
        
        #img1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
        #cv2.waitKey(1)
        #cv2.imshow("fff",img)
        if cv2.waitKey(1) & 0xFF == 27:
            break        

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    show_video()

