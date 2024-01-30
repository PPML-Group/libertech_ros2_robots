from ament_index_python.packages import get_package_share_path
import launch
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
# from launch.actions import DeclareLaunchArgument, LogInfo, PushRosNamespace, ExecuteProcess

from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression

from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_hand_driver(context, *args, **kwargs):
    # urdf_path = get_package_share_path('libertech_hand_description')
    urdf_file_name = 'hand_' + \
        launch.substitutions.LaunchConfiguration('hand_type').perform(context) + ".xacro.urdf"
    
    urdf_file = PathJoinSubstitution(
        [FindPackageShare("libertech_hand_description"), "urdf", urdf_file_name]
    )
    
    return [
        Node(
            package='libertech_hand_driver',
            executable='hand_ros_driver.py',
            name='hand_ros_driver',
            parameters=[{'urdf_file': urdf_file},
                        {'action_file': LaunchConfiguration("action_file")},
                        {'ip': LaunchConfiguration("hand_ip")}],
            output='screen',
        )
    ]


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
            "hand_ip",
            default_value='10.10.20.61',
            description="IP of Hand",
        )
    )
    
    action_file = PathJoinSubstitution(
        [FindPackageShare("libertech_hand_driver"), "config", "actions.yaml"]
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "action_file",
            default_value=action_file,
            description="action_file",
        )
    )
    
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("libertech_hand_description"), "urdf", "hand.xacro"]
            ),
            " ",
            "prefix:=",
            LaunchConfiguration("prefix"),
            " ",
            "type:=",
            LaunchConfiguration("hand_type"),
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
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_file],
    )
    
    rqt_node = Node(
        package='rqt_reconfigure',
        executable='rqt_reconfigure',
        name='rqt_reconfigure',
        output='screen',
    )
    
    nodes_to_start = [
        launch.actions.OpaqueFunction(function=launch_hand_driver),
        TimerAction(period=5.0, actions=[robot_state_publisher_node]),
        rviz_node,
        rqt_node,
        
    ]
    return LaunchDescription(declared_arguments + nodes_to_start)
