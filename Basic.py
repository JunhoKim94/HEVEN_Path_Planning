import numpy as np
from Lane_Detection import Line

class BasicPath:
    def __init__(self, left, right, howFar, width, height): 
        '''
        ==== Input ====
        left : 좌측 차선의 방정식
        right : 우측 차선의 방정식
        howFar : 얼마나 멀리 갈 것인지  ex) 400 = 800x600 이미지에서 세로 400까지 보겠다
        width : 이미지 가로
        height : 이미지 세로
        ==== Output ====
        destination : 목표점
        rotation : 기울기
        '''
        self.left = left
        self.right = right
        self.left_coef = self.left.current_fit
        self.right_coef = self.right.current_fit
        self.howFar = howFar
        self.width = width
        self.height = height

    def update(self, left, right): # 차선 식 업데이트
        self.left = left
        self.right = right

        self.left_coef = self.left.current_fit
        self.right_coef = self.right.current_fit

    def path(self):
        destination = 0
        rotation = 0
        y = self.height - self.howFar

        la = self.left_coef[0]
        lb = self.left_coef[1]
        lc = self.left_coef[2]
        left_x = ((y + (lb**2)/(4*la) - lc)/la)**0.5 - (lb/2/la)
        left_rot_x = 2*la*left_x + lb

        ra = self.right_coef[0]
        rb = self.right_coef[1]
        rc = self.right_coef[2]
        right_x = ((y + (rb**2)/(4*ra) - rc)/ra)**0.5 - (rb/2/ra)
        right_rot_x = 2*ra*right_x + rb

        destination = (left_x+right_x)/2.0
        rotation = (left_rot_x+right_rot_x)/2.0
    
        return destination, rotation


        
if __name__=='__main__': # Example
    
    left = Line()
    left.current_fit = [1., -6., 9.]
    right = Line()
    right.current_fit = [1., -10., 25.]
    #left_coef = {'current_fit':[1, 2, 1]}
    #right_coef = {'current_fit':[1, -4, 9]}
    
    # 사진 크기
    width = 800
    height = 2
    howFar = 1

    basic_path = BasicPath(left, right, howFar, width, height)
    destination, rotation = basic_path.path()

    print(destination, rotation)