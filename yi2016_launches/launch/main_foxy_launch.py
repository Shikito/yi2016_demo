import os
from datetime import datetime as dt

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    # Set Package Name
    package = 'yi2016'

    # Set Bag Dir
    tdatetime = dt.now()
    tstr = tdatetime.strftime('%Y%m%d%H%M%S')
    bag_dir = f'/home/toshi/ros_ws/bag/{package}/{tstr}'
    
    return LaunchDescription([
        Node(
            package=package,
            namespace=package,
            executable='control_manager',
            name='controller_manager'
        ),
        Node(
            package=package,
            namespace=package,
            executable='target_object',
            name='target_object_service'
        ),
        Node(
            package=package,
            namespace=package,
            executable='display_manager',
            name='display_manager'
        ),
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',
                '-a',
                '-b', '100000000', # 100MB
                '--compression-mode', 'file',
                '--compression-format', 'zstd',
                '--output', bag_dir],
            output='screen'
        ),
        ExecuteProcess(
            cmd=[
                'rqt',
                '-s', 'rqt_image_view/ImageView',
                '--args', f'/{package}/gp_result_graph'
                ],
            output='screen'
        )
    ])
