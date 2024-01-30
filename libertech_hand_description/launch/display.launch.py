from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description="prefix",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "hand_type",
            default_value="right",
            description="Type of hand",
            choices=["right", "left"],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value='true',
            description="Flag to enable joint_state_publisher_gu",
            choices=['true', 'false'],
        )
    )
    
    prefix = LaunchConfiguration("prefix")
    hand_type = LaunchConfiguration("hand_type")
    
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("libertech_hand_description"), "urdf", "hand.xacro"]
            ),
            " ",
            "prefix:=",
            prefix,
            " ",
            "type:=",
            hand_type,
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    rviz_file = PathJoinSubstitution(
        [FindPackageShare("libertech_hand_description"), "rviz", "urdf.rviz"]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description]
    )

    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_file],
    )

    nodes_to_start = [
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
