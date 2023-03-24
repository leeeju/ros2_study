# Create a node to convert quaternions to Euler angles
# Create : 2023.03.01
# Author : leeeju
# Refer : https://gist.github.com/salmagro/2e698ad4fbf9dae40244769c5ab74434?permalink_comment_id=3822361

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from rclpy.qos import QoSProfile
import math
"""
orientation x,y,z,w ----+
                                +--1---> +-------------------------+
input orientaion from_quaternion----->  |                         |
                                        | euler_from_quaternion() |
                                        |                         |
                               <--2---- +-------------------------+
            r,p,y,w angle <-----------+
"""
class Quaternion_To_Euler(Node):

    def __init__ (self):
        super().__init__ ('quaternion_to_euler')
        qos_profile = QoSProfile(depth=10)

        self.imu_sub = self.create_subscription(
            Imu,
            'imu',
            self.imu_callback,
            qos_profile )

        self.imu_pub = self.create_publisher(
            Vector3,
            'euler_angles',
            qos_profile )

    def imu_callback(self, msg):
        
        quaternion = (
            msg.orientation.w,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z)

        roll, pitch, yaw = self.quaternion_to_euler(quaternion)
        euler_msg = Vector3()
        euler_msg.x = roll
        euler_msg.y = pitch
        euler_msg.z = yaw
        self.publisher.publish(euler_msg)

    def quaternion_to_euler(self, quaternion):
        x = quaternion.x    #[1]
        y = quaternion.y    #[2]
        z = quaternion.Z    #[3]
        w = quaternion.w    #[0]

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)
    quaternion_to_euler = Quaternion_To_Euler()
    try:
        rclpy.spin(quaternion_to_euler)
    except KeyboardInterrupt:
        quaternion_to_euler.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        quaternion_to_euler.destroy_node()
        rclpy.shutdown()

if main == 'main':
    main()