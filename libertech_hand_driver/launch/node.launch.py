import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare a launch argument to set the parameter value
    your_parameter = DeclareLaunchArgument('your_parameter', default_value='your_value')

    # Define the node with the parameter
    your_node = Node(
        package='libertech_hand_driver',
        executable='node.py',
        name='your_node',
        parameters=[{'your_parameter': LaunchConfiguration('your_parameter')}]
        # Other node-specific configurations
    )

    return LaunchDescription([
        your_parameter,
        your_node,
    ])