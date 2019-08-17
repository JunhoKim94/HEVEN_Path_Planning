'''
TODO:    1. Restrict detections by size of the bounding boxes. Ex. if the the sign is too small (background) or too big (whole frame) it shoudlnt be considered a trustworthy information
         2. Parse data from detections by descending precision order
         3. Implement an algorithm using the certainty value -> mean, avg, deviation etc
         4. If its a red_light, pause detections and wait for a green_light signal. Make a function for detection? eg. detect(green_light) detect(all)
         5. Implement traffic light detection by using ROI and color extraction
            How to figure out the shape of the LED arrow? Preset histograms for each arrow and make comparison? cv2.compareHist
            How to draw boxes for each light? and as in:
                    ----------------------------------------
                    | Red 0 | Orange 1 | Arrow 2 | Green 3 |
                    ---------------------------------------
                    - Take histogram and find the brightness edges (similar as lane tracking techniques)
                    - Image Segmentation with Watershed Algorithm
         6. TRAIN AGAIN WITH 4 LIGHTS TRAFFIC LIGHT
         7. Set mission numbers on return
         8. Resolution

'''

from ctypes import *
import math
import random
import os
import cv2
import numpy as np
import time
import darknet
from traffic_hsv import lightsParser

# Parameters
SIZE = 20  # Buffer size (if a class reaches SIZE frames -> do mission)
IDLE_SIZE = 30  # Idle mode size (if no reading for IDLE frames -> reset buffer)
THRESH = 0.7  # YOLO threshold
MAX_SIZE = 170000  # Max area size that can be accepted as a detection   -> fix miss-detection with large bounding box
MIN_SIZE = 20  # Min area size that can be accepted as a detection   -> fix miss-detection with small bounding box and prevents detecting anything on the background

HEIGHT = 608
WEIGHT = 608
netMain = None
metaMain = None
altNames = None

font = cv2.FONT_HERSHEY_SIMPLEX
bottomLeftCornerOfText = (30, 30)
fontScale = 0.6
fontColor = (0, 0, 255)
lineType = 2


class YOLO():
    def __init__(self):
        self.definer = ['intersection',
                        'left',
                        'right',
                        'workers',
                        'bike',
                        'crosswalk',
                        'school',
                        'speed_bump',
                        'parking',
                        'bus',
                        'red_light',
                        'arrow_light',
                        'green_light']  # Define all 14 traffic signs used

        self.separate = ['workers',
                         'bike',
                         'crosswalk',
                         'school',
                         'speed_bump',
                         'parking',
                         'bus',
                         'red_light']

        self.sign = {key: [0, 0] for key in self.definer}  # Initialize a dict for each type to store a counter and precision checker

        # ---*Class*--------------------------------------------------------------------------------------------------------------------------------------------------------------------
        # |   intersection   |   left   |   right   |   workers   |    bike   |   crosswalk   |   school   |   speed_bump   |   parking   |   bus    |   red_light   |   green_light   |
        # ---*Counter*------------------------------------------------------------------------------------------------------------------------------------------------------------------
        # |        0         |     0    |     0     |      0      |     0     |       0       |      0     |        0       |      0      |    0     |       0       |        0        |
        # ---*Precision*----------------------------------------------------------------------------------------------------------------------------------------------------------------
        # |        0         |     0    |     0     |      0      |     0     |       0       |      0     |        0       |      0      |    0     |       0       |        0        |
        # ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

        # YOLO config
        global metaMain, netMain, altNames
        configPath = "custom/yolov3-t4.cfg"
        weightPath = "backup/yolov3-t4_best.weights"
        metaPath = "custom/obj.data"
        if not os.path.exists(configPath):
            raise ValueError("Invalid config path `" + os.path.abspath(configPath) + "`")
        if not os.path.exists(weightPath):
            raise ValueError("Invalid weight path `" + os.path.abspath(weightPath) + "`")
        if not os.path.exists(metaPath):
            raise ValueError("Invalid data file path `" + os.path.abspath(metaPath) + "`")
        if netMain is None:
            netMain = darknet.load_net_custom(configPath.encode("ascii"), weightPath.encode("ascii"), 0, 1)  # batch size = 1
        if metaMain is None:
            metaMain = darknet.load_meta(metaPath.encode("ascii"))
        if altNames is None:
            try:
                with open(metaPath) as metaFH:
                    metaContents = metaFH.read()
                    import re
                    match = re.search("names *= *(.*)$", metaContents, re.IGNORECASE | re.MULTILINE)
                    if match:
                        result = match.group(1)
                    else:
                        result = None
                    try:
                        if os.path.exists(result):
                            with open(result) as namesFH:
                                namesList = namesFH.read().strip().split("\n")
                                altNames = [x.strip() for x in namesList]
                    except TypeError:
                        pass
            except Exception:
                pass

    def convertBack(self, x, y, w, h):
        xmin = int(round(x - (w / 2)))
        xmax = int(round(x + (w / 2)))
        ymin = int(round(y - (h / 2)))
        ymax = int(round(y + (h / 2)))
        return xmin, ymin, xmax, ymax

    def cvDrawBoxes(self, detections, img):
        x1 = x2 = y1 = y2 = 0
        for detection in detections:
            x, y, w, h = detection[2][0], \
                         detection[2][1], \
                         detection[2][2], \
                         detection[2][3]
            xmin, ymin, xmax, ymax = self.convertBack(float(x), float(y), float(w), float(h))
            pt1 = (xmin, ymin)
            pt2 = (xmax, ymax)
            area = (xmax - xmin) * (ymax - ymin)

            if area < MAX_SIZE and area > MIN_SIZE:
                cv2.rectangle(img, pt1, pt2, (0, 0, 0), 1)
                cv2.putText(img, detection[0].decode() + " [" + str(round(detection[1] * 100, 2)) + "]",
                            (pt1[0], pt1[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, [0, 0, 0], 2)
                x1 = xmin
                x2 = xmax
                y1 = ymin
                y2 = ymax
        return img, x1, x2, y1, y2

    def checkSize(self, detection):
        if detection:
            x, y, w, h = detection[0][2][0], detection[0][2][1], detection[0][2][2], detection[0][2][3]
            xmin, ymin, xmax, ymax = self.convertBack(float(x), float(y), float(w), float(h))
            area = (xmax - xmin) * (ymax - ymin)
            if area < MAX_SIZE and area > MIN_SIZE:
                return (detection)

    def main(self):
        # cap = cv2.VideoCapture(1)
        # cap = cv2.VideoCapture('/dev/video2')
        cap = cv2.VideoCapture('real_yolo.avi')
        cap.set(3, WEIGHT)  # Feed resolution from webcam to YOLO however output is from yolo cfg file resolution
        cap.set(4, HEIGHT)
        out = cv2.VideoWriter("output.avi", cv2.VideoWriter_fourcc(*"MJPG"), 20.0, (darknet.network_width(netMain), darknet.network_height(netMain)))

        print("Starting the YOLO loop...")

        # Create an image we reuse for each detect
        darknet_image = darknet.make_image(darknet.network_width(netMain), darknet.network_height(netMain), 3)

        # Variables
        frame_count = countimg = 0
        turn = mx = frame_show = ''
        self.last_mission = ''
        self.TF_flag = [0, 0, 0]

        while True:
            self.mission = ''
            self.school = 0
            prev_time = time.time()
            ret, frame_read = cap.read()
            frame_rgb = cv2.cvtColor(frame_read, cv2.COLOR_BGR2RGB)

            frame_resized = cv2.resize(frame_rgb, (darknet.network_width(netMain),darknet.network_height(netMain)), interpolation=cv2.INTER_LINEAR) # CHECK RESIZING KEEPING RATIO

            darknet.copy_image_from_bytes(darknet_image, frame_resized.tobytes())

            detections = darknet.detect_image(netMain, metaMain, darknet_image, THRESH)

            detect = self.checkSize(detections)

            fps = (1 / (time.time() - prev_time))
            image, x1, x2, y1, y2 = self.cvDrawBoxes(detections, frame_resized)
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

            if detect:
                sg1 = detect[0][0].decode()
                prec1 = detect[0][1]
                # If there are 2 readings in a frame, parse both at the same time (fix for single parsing per frame. Only 2 because no need for 3+)
                if len(detect) == 2:
                    sg2 = detect[1][0].decode()
                    prec2 = detect[1][1]
                    if sg2 == 'red_light' or sg2 == 'green_light':
                        self.TF_flag = lightsParser(image, x1, x2, y1, y2)
                        if self.TF_flag is None:
                            pass
                        elif self.TF_flag[0] == 1:
                            self.sign['red_light'][0] += 1

                        elif self.TF_flag[1] == 1:
                            self.sign['arrow_light'][0] += 1

                        elif self.TF_flag[2] == 1:
                            self.sign['green_light'][0] += 1
                    else:
                        self.TF_flag = [0, 0, 0]
                        self.sign[str(sg2)][0] += 1
                        self.sign[str(sg2)][1] += prec2

                if sg1 == 'red_light' or sg1 == 'green_light':
                    self.TF_flag = lightsParser(image, x1, x2, y1, y2)
                    if self.TF_flag is None:
                        pass
                    elif self.TF_flag[0] == 1:
                        self.sign['red_light'][0] += 1

                    elif self.TF_flag[1] == 1:
                        self.sign['arrow_light'][0] += 1
                        print('arrow')

                    elif self.TF_flag[2] == 1:
                        self.sign['green_light'][0] += 1
                else:
                    self.TF_flag = [0, 0, 0]
                    self.sign[sg1][0] += 1
                    self.sign[sg1][1] += prec1

                mx = max(self.sign, key=lambda key: self.sign[key][0])
                if self.sign[mx][0]:
                    frame_show = str(self.sign[mx][0])
                frame_count = 0

            else:
                frame_count += 1  # In order to know IDLE, count frames without any reading and store in frame_count

            self.high_vals = {k: v for (k, v) in self.sign.items() if v[0] >= SIZE}  # Check if there are any values above size limit


            if self.high_vals:  # If there is any sign > SIZE
                max_sign = max(self.high_vals, key=lambda key: self.high_vals[key][0])  # Finds the key with highest frame_count on the dictionary
                max_precis = max(self.high_vals, key=lambda key: self.high_vals[key][1])  # Finds the key with highest precision on the dictionary
                counter = self.high_vals[max_sign][0]  # Finds the value of the highest frame_count on the dictionary
                precis = self.high_vals[max_precis][1]  # Finds the value of the highest precision on the dictionary

                if 'left' in self.high_vals and self.sign['left'][0] > self.sign['right'][0] and self.sign['left'][0] > self.sign['intersection'][0]:  # Left is the highest between right and front
                    turn = 'left'
                    self.mission = self.check_lights(turn)

                elif 'right' in self.high_vals and self.sign['right'][0] > self.sign['left'][0] and self.sign['right'][0] > self.sign['intersection'][0]:  # Right is the highest between left and front
                    self.mission = 'right'
                    self.reset_dict(0)

                elif 'intersection' in self.high_vals and self.sign['intersection'][0] > self.sign['right'][0] and self.sign['intersection'][0] > self.sign['left'][0]:  # Front is the highest between right and left
                    turn = 'front'
                    self.mission = self.check_lights(turn)

                elif max_sign == 'school':
                    self.mission = max_sign
                    self.school = 1
                    self.reset_dict(0)

                elif max_sign == 'green_light' or max_sign == 'red_light':  # Situation where car only sees traf. lights -> Go to last saved turn
                    self.mission = self.check_lights(turn)

                else:  # All other missions that dont use traffic lights
                    self.mission = max_sign
                    self.reset_dict(0)  # 0 = reset all 1 = reset all but left right inter green

                self.last_mission = self.mission  # Update last mission



            elif frame_count > IDLE_SIZE:  # If No detection for the last IDLE_SIZE frames -> refresh all  (used to avoid random uncertain and uncontinuos detections)
                self.reset_dict(0)
                mx = 'SEARCHING'
                frame_show = ''
                frame_count = 0
                self.last_mission = 'default'
                self.TF_flag = [0, 0, 0]

            m_number = self.get_mission_number()


            # DEBUGING: write frame count, last mission, the sign with max count and its count, fps
            cv2.putText(image, str(frame_count), bottomLeftCornerOfText, font, fontScale, fontColor, lineType)
            cv2.putText(image, self.last_mission, (450, 600), font, fontScale, fontColor, lineType)
            cv2.putText(image, mx, (400, 570), font, fontScale, fontColor, lineType)
            cv2.putText(image, frame_show, (550, 570), font, fontScale, fontColor, lineType)
            cv2.putText(image, str(m_number), (10, 370), font, fontScale, fontColor, lineType)
            cv2.putText(image, self.mission, (10, 400), font, fontScale, fontColor, lineType)
            cv2.putText(image, str(self.TF_flag), (10, 570), font, fontScale, fontColor, lineType)
            cv2.putText(image, 'fps:' + str(int(fps)), (550, 30), font, 0.5, fontColor, lineType)
            cv2.rectangle(image, (0, 0), (10, 10), (60, 60, 60), 1)
            cv2.rectangle(image, (0, 0), (316, 316), (60, 60, 60), 1)

            out.write(image) # P
            cv2.imshow('Demo', image)
            cv2.waitKey(3)

    def reset_dict(self, flag):
        if flag == 0:
            self.sign = {key: [0, 0] for key in self.definer}
        else:
            for i in self.separate:
                self.sign[i] = [0, 0]
        frame_count = 0

    def check_lights(self, turn):
        if (self.sign['red_light'][0] > self.sign['green_light'][0]) and self.sign['red_light'][0] > 5:  # *Car to-do*: Reset the buffer (not left and right), find the line and stop. **add better implementation
            if self.sign['arrow_light'][0] > 5 and turn == 'left':
                self.mission = 'left'
                self.reset_dict(0)
            else:
                self.mission = turn + ' - TL'
                self.reset_dict(1)
                self.sign['red_light'][0] = 6
        else:  # *Car to-do*: Green lights - MOVE
            self.mission = turn
            self.reset_dict(0)
        return (self.mission)

    # Return mission number, easy call
    def get_mission_number(self):
        if self.last_mission == 'workers':
            m_number = 1

        elif self.last_mission == 'bike':
            m_number = 2

        elif self.last_mission == 'left':
            m_number = 3

        elif self.last_mission == 'front':
            m_number = 4

        elif self.last_mission == 'right':
            m_number = 5

        elif self.last_mission == 'left - TL':
            m_number = 6

        elif self.last_mission == 'front - TL':
            m_number = 7

        elif self.last_mission == 'parking':
            m_number = 8

        elif self.last_mission == 'school':
            self.school = 1

        elif self.last_mission == 'bus':
            m_number = 9

        else:
            m_number = 0
        return m_number

    # Return traffic lights status, easy call
    def traffic_light(self):
        return self.TF_flag  # [red, arrow, green, stop]

    # Return either in school zone or not, easy call
    def is_school_zone(self):
        return self.school


if __name__ == '__main__':
    yo = YOLO()
    yo.main()