import cv2
import numpy as np
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
max_pixel_num = 100
#최소 픽셀 수
min_pixel_num = 30
#회귀를 위한 최소 라인 포인트 수
min_points_num = 4
#색상 범위 HSV
boundaries = [
    (np.array([161, 155, 84], dtype="uint8"), np.array([179, 255, 255], dtype="uint8")), # red1
    (np.array([0, 100, 70], dtype="uint8"), np.array([20, 255, 255], dtype="uint8")), # red2
    (np.array([94, 80, 200], dtype="uint8"), np.array([126, 255, 255], dtype="uint8")), # blue
    (np.array([10, 30, 50], dtype="uint8"), np.array([25, 255, 255], dtype="uint8")), #yellow
    (np.array([0, 0, 180], dtype="uint8"), np.array([180, 25, 255], dtype="uint8")) # white
]
# 모니터링 창 크기
display = (640, 480)
# 클로징 마스크 크기
kernel = np.ones((7,7), np.uint8)
class Line:
    def __init__(self,current_line = None,curve = 0,detect = False):
        self.current_fit = [np.array([False])]
        self.prevx = []
        self.allx = None
        self.ally = None
        self.curve = curve
        self.detect = detect
        # polynomial coefficients for the most recent fit
        self.current_fit = [np.array([False])]

## Lane_Detection.py
class Lane_Detection: #Lane_Detction 클래스 생성후, original img 변경
    def __init__(self, img,left,right):  # 초기화
        self.original_img=img
        
        self.left = left
        self.right = right
        

        self.height, self.width = img.shape[:2]
        #roi설정을 위한 vertics, 위부터 차례대로 왼쪽 위, 왼쪽 아래, 오른쪽 아래, 오른쪽 위다.
        
        self.vertics = np.array([[(int(0.4*self.width), int(0.05*self.height)), 
                                  (int(-2*self.width), int(0.8*self.height)),
                                  (int(3*self.width), int(0.8*self.height)),
                                  (int(0.6*self.width), int(0.05*self.height))]])
    
    
        '''
        self.vertics = np.array([[(int(0.3*self.width), int(0.5*self.height)), 
                                  (int(0*self.width), int(0.9*self.height)),
                                  (int(1*self.width), int(0.9*self.height)),
                                  (int(0.7*self.width), int(0.5*self.height))]])    
        
        '''
        #perspective변환을 위한 pts 설정
        
        self.pts1 = np.float32([(0.45*self.width, 0.07*self.height), 
                                (0.1*self.width, 0.4*self.height),
                                (0.9*self.width, 0.4*self.height),
                                (0.55*self.width, 0.07*self.height)])
        '''
        self.pts1 = np.float32([(0*self.width, 0.6*self.height),
                                (0.1*self.width, 0.9*self.height),
                                (1*self.width, 0.9*self.height),
                                (0.6*self.width, 0.6*self.height)])
        '''
        self.temp1, self.temp2 = display[:2]
        self.pts2 = np.float32([(0.3*self.temp1, 0.2*self.temp2),
                                (0.3*self.temp1, 0.8*self.temp2),
                                (0.7*self.temp1, 0.8*self.temp2),
                                (0.7*self.temp1, 0.2*self.temp2)])

        self.binary_img = self.make_binary()
        
        self.bin_height, self.bin_width = self.binary_img.shape[:2]
        cv2.imshow('bin', self.binary_img)
        
        self.map = self.search_lines(self.binary_img)
        
    def smoothing(self,lines, pre_lines=3):
        # collect lines & print average line
        lines = np.squeeze(lines)
        avg_line = np.zeros((480))
    
        for ii, line in enumerate(reversed(lines)):
            if ii == pre_lines:
                break
            avg_line += line
        avg_line = avg_line / pre_lines
    
        return avg_line
    
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
        
        
        lx = np.concatenate(left_lane_x)
        ly = np.concatenate(left_lane_y)
        rx = np.concatenate(right_lane_x)
        ry = np.concatenate(right_lane_y)

        self.left.current_line = [lx,ly]
        self.right.current_line = [rx,ry]
        
        ploty = np.linspace(0, b_img.shape[0] - 1, b_img.shape[0])
        
        '''
        ransac1 = linear_model.RANSACRegressor()
        ransac2 = linear_model.RANSACRegressor()
        ransac1.fit(self.add_square_feature(ly),lx)
        ransac2.fit(self.add_square_feature(ry),rx)
        y1 = np.round(ransac1.predict(self.add_square_feature(ploty)))
        y2 = np.round(ransac2.predict(self.add_square_feature(ploty)))
        '''
        
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
            left_avg_line = self.smoothing(self.left.prevx, num)
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
            right_avg_line = self.smoothing(self.right.prevx, num)
            right_avg_fit = np.polyfit(ploty, right_avg_line, poly_order)
            r = np.poly1d(right_avg_fit)
            right_fit_plotx = r(ploty)
            self.right.current_fit = right_avg_fit
            self.right.allx, self.right.ally = right_fit_plotx, ploty
        else:
            self.right.current_fit = right_fit
            self.right.allx, self.right.ally = y2, ploty
        
        self.draw_points(monitor,self.left.allx,ploty,(0,0,255),3)
        self.draw_points(monitor,self.right.allx,ploty,(0,255,0),3)

        cv2.imshow("ss",monitor)
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
        for_test = self.warp_image(img)
        cv2.imshow("for_test", for_test)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        img = self.Detect(img)
        #img = self.closeimage(img)
        img = self.warp_image(img)
              
        return img
    
    def add_square_feature(self,X):
        X = np.array(X)
        X = X.reshape(-1,1)
        X = np.concatenate([(X**2).reshape(-1,1), X], axis=1)
        return X
    
    def reg_of_int(self, img): # 이미지에서 roi 잘라내기
        self.mask = np.zeros_like(img)
        cv2.fillPoly(self.mask, self.vertics, (255,255,255))
        self.mask = cv2.bitwise_and(img, self.mask)
        return self.mask
        
    def warp_image(self, img): # 이미지 원근 변환
        M = cv2.getPerspectiveTransform(self.pts1, self.pts2)
        warped_img = cv2.warpPerspective(img, M, display,flags=cv2.INTER_AREA)
        return warped_img
    
    def Detect(self,img):
        #th_sobelx, th_sobely, th_mag, th_dir = (35, 100), (30, 255), (30, 255), (0.7, 1.3)
        #th_h, th_l, th_s = (10, 100), (0, 60), (85, 255)
        #combined_gradient = gradient_combine(img, th_sobelx, th_sobely, th_mag, th_dir)
        combined_hsv = cv2.cvtColor(np.zeros_like(img), cv2.COLOR_BGR2GRAY)
        
        for color in ['w', 'y', 'b']:
            detected = self.detectcolor(img, color)
            combined_hsv = cv2.bitwise_or(combined_hsv, detected)
            
        #blur_img = cv2.GaussianBlur(img, (7,7), 10)
        #L2 정규화 추가로 noise 제거
        img2 = cv2.cvtColor(img,cv2.COLOR_HSV2BGR)
        canny_img = cv2.Canny(img2, 150, 230, edges = None, apertureSize = 3,L2gradient=True)
        combined_result = comb_result(canny_img, combined_hsv)
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
    video="./video/pre_lane_Trim.mp4"
    cap = cv2.VideoCapture(video)
    
    left = Line()
    right = Line()
    
    while True:
        ret, img = cap.read()
        if not ret:
            print('비디오 끝')
            break
        img1 = Lane_Detection(img, left, right)
        cv2.imshow("adfa",img1.mask)
        cv2.imshow("zzz",img)
        
        if cv2.waitKey(1) & 0xFF == 27:
            break        

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    show_video()

