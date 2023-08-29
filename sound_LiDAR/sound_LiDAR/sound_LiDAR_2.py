#!/usr/bin/env python3
# -*- coding: utf-8 -*- #
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import LaserScan as LiDAR
import numpy as np
import math

class clustering(Node):
    def __init__(self):
        super().__init__('clustering_node')
        self.x_LiDAR        = 200
        self.y_LiDAR        = 300
        self.cnt            = 0
        self.distance_data  = []
        self.distance       = 0
        self.RC_group = ReentrantCallbackGroup()
        self.create_subscription(LiDAR,'/scan',self.LiDAR_callback,1, callback_group=self.RC_group)

    def LiDAR_callback(self,msg):
        
        self.converted_data = []
        self.lidar_data = msg.ranges
        self.lidar_angle_increment = msg.angle_increment
        for i in range(len(self.lidar_data)):
            angle = (self.lidar_angle_increment * i)
            x = -np.sin(angle) * self.lidar_data[i] * 1000
            y = np.cos(angle) * self.lidar_data[i] * 1000

            if (abs(x) < self.x_LiDAR) & (y > 0) & (y < self.y_LiDAR): #mm
                self.cnt +=1
                self.converted_data.append([x,y,self.cnt])
                self.distance = (math.sqrt(x*x + y*y))
                self.distance_data.append(self.distance)
                # print(f"{(self.distance_data)}")
                for i in range(len(self.distance_data)):
                    if abs(self.distance_data[i] - self.distance_data[i-1]) > 100:
                        print(f'{self.distance_data}')
                    else:
                        pass

def main():
    rclpy.init()

    C = clustering()

    try:
        rclpy.spin(C)

    finally:
        C.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()