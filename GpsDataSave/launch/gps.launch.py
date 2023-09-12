from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='GpsDataSave',
            executable='gps_data_save',
            name='gps_data_save',
            stdout_linebufered=True,
            remappings=[
                ('/gps_data', '/gps_data')
            ]
        )
    ])