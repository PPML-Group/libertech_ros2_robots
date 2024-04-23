from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='libertech_humanoid_bringup',  # 替换成你的包名
            executable='change_pose.py',
            name='change_pose_node',
            output='screen',
        ),
    ])
