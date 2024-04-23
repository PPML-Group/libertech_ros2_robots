from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='libertech_humanoid_bringup',
            executable='pose_publisher.py',
            name='pose_publisher',
            output='screen',
        )
    ])