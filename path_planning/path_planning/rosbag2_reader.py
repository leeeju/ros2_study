#!/usr/bin/env python3
# -*- coding: utf-8 -*- #
import rclpy
from sbg_driver.msg import SbgGpsPos
from rclpy.node import Node
import numpy as np
import math
import utm
class Text_Reader(Node):
    def __init__(self):
        super().__init__('Text_Reader')
        self.subscription = self.create_subscription(SbgGpsPos,'/sbg/gps_pos',self.gps_callback,10)
        self.utm_x=[]
        self.utm_y=[]

    def gps_callback(self, msg):
        self.gps_x = float(msg.latitude)
        self.gps_y = float(msg.latitude)
        self.utm_position = utm.from_latlon(self.gps_x,self.gps_y)
        self.utm_x.append(self.utm_position[0])
        self.utm_y.append(self.utm_position[1])
        self.save_path('/home/labmas/robot_ws/src/j_LiDAR/path', "gps", self.utm_x, self.utm_y)

    def save_path(self,directory, sensor_name, x = [], y = []):
        f_x   = open(directory + "/" + sensor_name + "_x.txt",   'w')
        f_y   = open(directory + "/" + sensor_name + "_y.txt",   'w')

        for i in range(0, len(x)-1):
            f_x.write(str(x[i]) + '\n')
            f_y.write(str(y[i]) + '\n')

        f_x.write(str(x[-1]))
        f_y.write(str(y[-1]))

        f_x.close()
        f_y.close()

def main(args=None):
    rclpy.init(args=args)
    bag_to_text = Text_Reader()
    try:
        rclpy.spin(bag_to_text)
    except KeyboardInterrupt:
        pass
    finally:
        bag_to_text.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()