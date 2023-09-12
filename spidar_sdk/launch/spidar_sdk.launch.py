import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    return LaunchDescription([
        Node(
            package='spidar_sdk',
            executable='spidar_sdk_node',
            namespace='spidar_sdk',
            output='screen'
            ),
        ])
