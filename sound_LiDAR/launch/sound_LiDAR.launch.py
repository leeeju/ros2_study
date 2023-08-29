
import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='sound_LiDAR',
                executable='sound_LiDAR',
                name='sound_LiDAR',
                output='screen',
                emulate_tty=True,
                parameters=[
                    {"x_LiDAR": 50},
                    {"y_LiDAR": 300},
                ],
            ),
        ]
    )