import sys
import os
sys.path.append(os.path.dirname(__file__))

from .CAM import CAM
from .Flag import Flag
from .GPS import GPS
from .LiDAR import LiDAR
from .Platform import Platform
from .imu import Imu
from .Screens import Screens
from .Path import Path

import threading
import time

class Database:
    def __init__(self, gps=True, platform=True, cam=True, lidar=True, imu=True):
        self.__gps_on = False
        self.__platform_on = True
        self.__cam_on = False
        self.__lidar_on = False
        self.__imu_on = False

        self.flag = Flag()
        self.path = Path()
        self.screen = Screens()
        print("\nIntializing Database")
        print("──────────────────────────────────────────────────────────────────────────────────────────────────────────────────")
        if self.__gps_on:
            self.gps = GPS('COM4', 19200, flag=self.flag)
            self.__gps_thread = threading.Thread(target=self.gps.main)

        if self.__platform_on:
            self.platform  = Platform('COM3', 115200, flag=self.flag)
            self.__platform_thread = threading.Thread(target=self.platform.main)

        if self.__cam_on:
            self.main_cam = CAM(0, 'Main', flag=self.flag)
            self.sub_cam = CAM(1, 'Sub', flag=self.flag)
            self.__main_cam_thread = threading.Thread(target=self.main_cam.main)
            self.__sub_cam_thread = threading.Thread(target=self.sub_cam.main)

        if self.__lidar_on:
            self.lidar = LiDAR('169.254.248.220', 2111, 57600, flag=self.flag)
            self.__lidar_thread = threading.Thread(target=self.lidar.main)

        if self.__imu_on:
            self.imu = Imu("COM3", 115200, flag=self.flag)
            self.__imu_thread = threading.Thread(target=self.imu.main)

        print("──────────────────────────────────────────────────────────────────────────────────────────────────────────────────")
        print("Database is ready to run!")

    def start(self):
        print("\nStart to run Database...")
        print("──────────────────────────────────────────────────────────────────────────────────────────────────────────────────")
        if self.__gps_on:
            self.__gps_thread.start()
            time.sleep(0.1)

        if self.__platform_on:
            self.__platform_thread.start()
            time.sleep(0.1)

        if self.__cam_on:
            self.__main_cam_thread.start()
            time.sleep(0.1)
            self.__sub_cam_thread.start()
            time.sleep(0.1)
        
        if self.__lidar_on:
            self.__lidar_thread.start()
            time.sleep(0.1)
        
        if self.__imu_on:
            self.__imu_thread.start()
            time.sleep(1)
        print("──────────────────────────────────────────────────────────────────────────────────────────────────────────────────")
        print("Database is running!\n")
    
    def join(self):
        print("\nTerminating Database...")
        print("──────────────────────────────────────────────────────────────────────────────────────────────────────────────────")
        if self.__gps_on:
            self.__gps_thread.join()

        if self.__platform_on:
            self.__platform_thread.join()

        if self.__cam_on:
            self.__main_cam_thread.join()
            self.__sub_cam_thread.join()
        
        if self.__lidar_on:
            self.__lidar_thread.join()
            time.sleep(0.1)
        
        if self.__imu_on:
            self.__imu_thread.join()
        print("──────────────────────────────────────────────────────────────────────────────────────────────────────────────────")
        print("Database termination complete!")


