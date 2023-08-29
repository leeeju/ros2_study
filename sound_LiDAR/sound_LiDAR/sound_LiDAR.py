#!/usr/bin/env python3
# -*- coding: utf-8 -*- #
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import LaserScan as LiDAR
from ament_index_python import get_package_share_directory
import numpy as np
import math
import pygame
import os

class clustering(Node):
    def __init__(self):
        super().__init__('clustering_node')
        self.x_LiDAR        = 50
        self.y_LiDAR        = 300
        self.cnt            = 0
        self.distance       = 0

        self.create_subscription(LiDAR, '/scan', self.LiDAR_callback, 10, callback_group=ReentrantCallbackGroup())

        # package_name = 'j_LiDAR'
        # package_path = get_package_share_directory(package_name)
        # self.audio_file_path = os.path.join(package_path, 'config', 'aaaa.mp3')
        self.audio_file_path = '/home/turtle/robot_ws/src/j_LiDAR/config/aaaa.mp3'

        pygame.mixer.init()

    def LiDAR_callback(self, msg):
        self.distance_data  = []
        self.converted_data = []
        self.lidar_data = msg.ranges
        self.lidar_angle_increment = msg.angle_increment
        for scan in range(len(self.lidar_data)):
            x = self.lidar_data[scan] * 500 * math.cos(scan * self.lidar_angle_increment)
            y = self.lidar_data[scan] * 500 * math.sin(scan * self.lidar_angle_increment)

            if (abs(x) < self.x_LiDAR) and (y > 0) and (y < self.y_LiDAR):  # mm
                self.cnt += 1
                self.converted_data.append([x, y])
                self.distance = math.sqrt(x * x + y * y)
                self.distance_data.append(self.distance)
                print(f"Distance {scan + 1}: {self.distance:.2f} mm")

            if abs(self.distance - 250) < 1: # mm

                if os.system(self.audio_file_path):
                    if os.path.exists(self.audio_file_path):
                        sound = pygame.mixer.Sound(self.audio_file_path)
                        sound.play()
                        print("Play audio file")

def main():
    rclpy.init(args=None)
    LiDAR = clustering()
    try:
        rclpy.spin(LiDAR)
    finally:
        LiDAR.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
