from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='yi2016',
            namespace='yi2016',
            executable='control_manager',
            name='controller_manager'
        ),
        Node(
            package='yi2016',
            namespace='yi2016',
            executable='target_object',
            name='target_object_service'
        ),
        Node(
            package='yi2016',
            namespace='yi2016',
            executable='display_manager',
            name='display_manager'
        )
    ])