from typing import Iterator
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.subscription import Subscription
from sensor_msgs.msg import NavSatFix

import time
import threading
import csv
import os
import datetime

class GpsDataSaveNode(Node):
    def __init__(self):
        super().__init__('gps_data_save_node')  # Node 이름을 'my_node'로 초기화
        qos_profile = QoSProfile(depth=10)
        self.callback_group = ReentrantCallbackGroup()

        self.subscription = self.create_subscription(
            NavSatFix,
            '/imu/nav_sat_fix',
            self.listener_callback,
            qos_profile,
            callback_group=self.callback_group)

        self.subscription  # prevent unused variable warning

        self.is_ready = False

        # CSV 파일을 저장할 경로를 지정
        self.data_path = os.path.join(os.path.expanduser('~'), 'data')

        # If the directory does not exist, create it
        if not os.path.exists(self.data_path):
            os.makedirs(self.data_path)

        # 다음에 저장될 파일의 이름을 지정
        self.csv_file_name = self.get_file_name()

        self.executor_ = MultiThreadedExecutor(num_threads=4)
        self.executor_.add_node(self)
        self.spin_thread = threading.Thread(target=self.spin)
        self.spin_thread.start()
        # CSV 파일을 생성하고 헤더를 작성
        with open(os.path.join(self.data_path, self.csv_file_name), 'w') as file:
            writer = csv.writer(file)
            writer.writerow(['timestamp', 'latitude', 'longitude', 'sd_x', 'sd_y', 'sd_yaw'])  # sd : standard_deviation

    def get_file_name(self):
        """파일의 고유한 이름을 가져옵니다."""
        now = datetime.datetime.now()  # 현재의 날짜와 시간을 얻음
        file_name = now.strftime("gps_data_%Y_%m_%d_%H_%M_%S.csv")  # 파일 이름 형식을 지정함
        return file_name

    def listener_callback(self, msg):
        """메시지 콜백."""
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        latitude = msg.latitude
        longitude = msg.longitude
        standard_deviation_x, standard_deviation_y, standard_deviation_yaw = msg.position_covariance[0], msg.position_covariance[4], msg.position_covariance[8]

        standard_deviation_x = standard_deviation_x**(1/2)
        standard_deviation_y = standard_deviation_y**(1/2)
        standard_deviation_yaw = standard_deviation_yaw**(1/2)

        # Write data to CSV file
        with open(os.path.join(self.data_path, self.csv_file_name), 'a') as file:
            writer = csv.writer(file)
            writer.writerow([timestamp, latitude, longitude, standard_deviation_x, standard_deviation_y, standard_deviation_yaw])

    def spin(self):
        try:
            if not self.is_ready:
                self.executor_.spin()
                self.spin_thread.start()
        except KeyboardInterrupt:
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = GpsDataSaveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
