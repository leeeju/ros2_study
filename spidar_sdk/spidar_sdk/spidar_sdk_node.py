#!/usr/bin/env python3

from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from rclpy.executors import MultiThreadedExecutor

import threading
import csv
import datetime
import os
import time
import rclpy
import socket
import struct
import json
import requests
import numpy as np


IP_ADDRESS = '192.168.20.221'
# Commands and URLs
API_URL = "http://" + IP_ADDRESS + ":8080/api"
NIC_SYSTEM_INFO_CMD = API_URL + "/nic/system_information"
GPR_SYSTEM_INFO_CMD = API_URL + "/nic/gpr/system_information"
DATA_SOCKET_CMD = API_URL + "/nic/gpr/data_socket"
POWER_CMD = API_URL + "/nic/power"
SETUP_CMD = API_URL + "/nic/setup"
ACQUISITION_CMD = API_URL + "/nic/acquisition"
# Other constants
POINTS_PER_TRACE = 200
TIME_SAMPLING_INTERVAL_PS = 100
POINT_STACKS = 4
PERIOD_S = 1
FIRST_BREAK_POINT = 20

# Configuration for the following put commands
POWER_ON_CONFIGURATION = {"data": json.dumps({'state': 2})}
START_ACQUISITION_CONFIGURATION = {"data": json.dumps({'state': 1})}
STOP_ACQUISITION_CONFIGURATION = {"data": json.dumps({'state': 0})}

# Constants for parsing the GPR data
HEADER_SIZE_BYTES = 20
POINT_SIZE_BYTES = 4


class SDKNode(Node):
    def __init__(self):
        super().__init__('sdk_node')
        self.get_logger().info("Sdk Node Started")
        self.data_publisher = self.create_publisher(Int32MultiArray, 'sdk_data', 10)
        # Create a directory to store the data
        self.data_path = os.path.join(os.path.expanduser('~'), 'sdk_data')
        if not os.path.exists(self.data_path):
            os.makedirs(self.data_path)
        self.csv_file_name = datetime.datetime.now().strftime('sdk_%Y%m%d_%H%M%S') + '.csv'
        self.csv_file_path = os.path.join(self.data_path, self.csv_file_name)
        self.init_csv_file()
        # Create a MultiThreadedExecutor to run the node in a separate thread
        self.executor_ = MultiThreadedExecutor(num_threads=4)
        self.executor_.add_node(self)
        self.spin_thread = threading.Thread(target=self.spin)
        self.spin_thread.start()
        self.rate = self.create_rate(1)

    def init_csv_file(self):
        if not os.path.exists(self.csv_file_path):
            with open(self.csv_file_path, 'w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['tv_sec', 'tv_nsec', 'trace_num', 'status', 'stacks', 'header_size'])

    def log_data(self, data):
        with open(self.csv_file_path, 'a', newline='') as file:
            writer2 = csv.writer(file)
            writer2.writerow(data)

    def get_requests(self, command, command_str_name, data=None):
        try:
            response = requests.put(command, data=data)
            json_response = json.loads(response.content)
            self.get_logger().info("Response from {} command: {}\n".format(command_str_name, json_response["status"]["message"]))
        except ValueError as err:
            self.get_logger().info("Unable to decode JSON: {}\n".format(err))

        if json_response["status"]["status_code"] != 0:
            self.get_logger().info("Command failed: {}".format(json_response["status"]["message"]))
            json_response = None
        return json_response

    def put_requests(self, command, data, command_str_name):

        # Perform a put request to change the resource’s data.
        # @param command: URL for the request command
        # @type command: str
        # @param data: Data to send to the command
        # @type data: dict
        # @param command_str_name: Name of the command
        # @type command_str_name: str
        # @return: response in json format
        # @rtype: JSON or None when request failed

        json_response = None
        try:
            response = requests.put(command, data=data)
            json_response = json.loads(response.content)
            self.get_logger().info("Response from {} command: {}\n".format(command_str_name, json_response["status"]["message"]))
        except ValueError as err:
            print("Unable to decode JSON: {}\n".format(err))
        if json_response["status"]["status_code"] != 0:
                self.get_logger().info("Command failed: {}".format(json_response["status"]["message"]))
                json_response = None
        # return json_response

        nic_system_information_response = self.get_requests(NIC_SYSTEM_INFO_CMD, "NIC's System Information")
        nic_system_info_response_data = nic_system_information_response['data']
        # Print the NIC500 system information
        print(("System Information:\nApp DIP: {}\nApp Version: {}\nFPGA Version: {}\nHardware ID: {}\nKernel Version: {}\n"
        "NIC Serial Number: {}\nOS DIP: {}\nOS Version: {}\nSmcApi: {} \n").format(
          nic_system_info_response_data['app_dip'],           nic_system_info_response_data['app_version']
        , nic_system_info_response_data['app_dip'],           nic_system_info_response_data['fpga_version']
        , nic_system_info_response_data['kernel_version'],    nic_system_info_response_data['hardware_id']
        , nic_system_info_response_data['nic_serial_number'], nic_system_info_response_data['os_dip']
        , nic_system_info_response_data['os_version'],        nic_system_info_response_data['smc_api_build']))

            # Use the NIC Power command to turn on the GPR
        power_on_response = self.get_requests(POWER_CMD, POWER_ON_CONFIGURATION, "Power")
            # Confirm the GPR was able to turn on
        if power_on_response is None:
                quit()

        # Use the GPR System Information command to get system information
        gpr_system_json_response = self.get_requests(GPR_SYSTEM_INFO_CMD, "GPR System Information")
        # Confirm we've received a successful response
        if gpr_system_json_response is None:
            quit()

        # Print out GPR system info
        gpr_system_info = gpr_system_json_response['data']['gpr']
        print((
            "GPR System Information:\nApp Code DIP: {}\nApp Code Version: {}\nDEV0:\n\tApp Code DIP: {}\n\tType: {}\n\t"
            "Unit Serial Number: {}\nDEV1:\n\tApp Code DIP: {}\n\tType: {}\n\tUnit Serial Number: {}\nGIC Serial Number: {}\n"
            "Unit Serial Number: {}\nWindow Time Shift Reference (ps): {}\n").format(
            gpr_system_info['app_code_dip'], gpr_system_info['app_code_version']
            , gpr_system_info['dev0']['app_code_dip'], gpr_system_info['dev0']['type']
            , gpr_system_info['dev0']['unit_serial_number'], gpr_system_info['dev1']['app_code_dip']
            , gpr_system_info['dev1']['type'], gpr_system_info['dev1']['unit_serial_number']
            , gpr_system_info['gic_serial_number'], gpr_system_info['unit_serial_number']
            , gpr_system_info['window_time_shift_reference_ps']))
        # Set the window time shift based on the reference value from GPR system information,
        # so that first break appears is at FIRST_BREAK_POINT (#20 in this example). If it doesn't exist use
        # the default value.
        window_time_shift_ps = -55000
        if gpr_system_info['window_time_shift_reference_ps'] is not None:
            window_time_shift_reference_ps = int(gpr_system_info['window_time_shift_reference_ps'])
            window_time_shift_ps = window_time_shift_reference_ps - (FIRST_BREAK_POINT * TIME_SAMPLING_INTERVAL_PS)
        # Use the GPR data socket command to get data socket's port
        data_socket_json_response = self.get_requests(DATA_SOCKET_CMD, "GPR Data Socket")
        # Get the data socket port
        data_socket_port = data_socket_json_response['data']['data_socket']['port']
        # Use the port to connect to the data socket
        data_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        data_socket.connect((IP_ADDRESS, data_socket_port))
        # Create configuration struct for NIC setup command
        setup_nic_configuration = {'data': json.dumps(
            {"gpr0": {"parameters": {'points_per_trace': POINTS_PER_TRACE,
                                    'window_time_shift_ps': window_time_shift_ps,
                                    'point_stacks': POINT_STACKS,
                                    'time_sampling_interval_ps': TIME_SAMPLING_INTERVAL_PS
                                    }},
            "timer": {"parameters": {"period_s": PERIOD_S}}})}
        # Use the NIC Setup command to setup the gpr and timer
        setup_nic_json_response = self.put_requests(SETUP_CMD, setup_nic_configuration, "Setup")
        # Confirm we've received a successful response
        if setup_nic_json_response is None:
            data_socket.close()
            quit()

        # Use the GPR data acquisition command to start data collection
        start_acquisition_json_response = self.put_requests(ACQUISITION_CMD, START_ACQUISITION_CONFIGURATION, "Start Acquisition")
        # Confirm we've received a successful response
        if start_acquisition_json_response is None:
            data_socket.close()
            quit()

        gpr_data = b''
        gpr_data_size_bytes = HEADER_SIZE_BYTES + (POINTS_PER_TRACE * POINT_SIZE_BYTES)
        num_traces_received = 0
        trace_num = 0
        ## Continue printing traces until we receive 10
        while trace_num < 10000:
            gpr_data = gpr_data + data_socket.recv(gpr_data_size_bytes)
            # Calculate the number traces received by the socket
            num_traces_received = int(len(gpr_data) / gpr_data_size_bytes)

            for i in range(0, num_traces_received):
                (tv_sec, tv_nsec, trace_num, status, stacks, header_size), s = struct.unpack('<LLLLHH',
                                                                                            gpr_data[0:HEADER_SIZE_BYTES]), \
                                                                            gpr_data[HEADER_SIZE_BYTES:gpr_data_size_bytes]
                gpr_data = gpr_data[gpr_data_size_bytes:]
                print("Trace Header: tv_sec: {} tv_nsec: {} trace_num: {} status: {} stacks: {} header_size: {} ".format(tv_sec,
                                                                                                                        tv_nsec,
                                                                                                                        trace_num,
                                                                                                                        status,
                                                                                                                        stacks,
                                                                                                                        header_size))
                first_ten_trace_points = np.frombuffer(s[0:40], dtype=np.float32)
                print("First ten points (mV): {} \n".format(first_ten_trace_points))
                # Publish the data as a ROS 2 message
                data_msg = Int32MultiArray(data=[tv_sec, tv_nsec, trace_num, status, stacks, header_size])
                time.sleep(1)
                self.data_publisher.publish(data_msg)

            # Use the GPR data acquisition command to stop data collection
            self.put_requests(ACQUISITION_CMD, STOP_ACQUISITION_CONFIGURATION, "Stop Acquisition")
            # Close data socket
            data_socket.close()


    def spin(self):
        try:
            self.executor_.spin()
            self.rate.sleep()
            self.spin_thread.start()
        except KeyboardInterrupt:
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    gpr_node = SDKNode()
    try:
        rclpy.spin(gpr_node)
    except KeyboardInterrupt:
        pass
    gpr_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()