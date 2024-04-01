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
            executable='hand_demo.py',
            name='hand_demo_'+launch.substitutions.LaunchConfiguration('hand_type').perform(context),
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
    
    nodes_to_start = [
        launch.actions.OpaqueFunction(function=launch_hand_driver),
    ]
    return LaunchDescription(declared_arguments + nodes_to_start)
