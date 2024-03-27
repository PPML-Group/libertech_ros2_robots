from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    TimerAction,
    IncludeLaunchDescription
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    hand_type = LaunchConfiguration("hand_type")
    hand_ip = LaunchConfiguration("hand_ip")
    action_file = LaunchConfiguration("action_file")
    sim = LaunchConfiguration("sim")
    
    hand_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("libertech_hand_driver"), "/launch/hand_ros_driver.launch.py"]
        ),
        launch_arguments={
            "hand_type": hand_type,
            "hand_ip": hand_ip,
            "action_file": action_file,
            "sim": sim,
        }.items()
    )
    
    urdf_file_gen_name = 'hand_' + hand_type.perform(context) + '.xacro.urdf'
    urdf_file_gen = urdf_file_gen = PathJoinSubstitution(
        [FindPackageShare("libertech_hand_description"), "urdf", f"{urdf_file_gen_name}"]
    )

    hand_retarget = Node(
        package='libertech_hand_driver',
        executable='hand_retarget_mediapipe.py',
        name='hand_retarget_mediapipe',
        parameters=[{'urdf_file': urdf_file_gen}],
        output='screen',
    )

    nodes_to_start = [
        hand_driver_launch,
        hand_retarget,
    ]
    
    return nodes_to_start


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
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim",
            default_value='True',
            description="Sim",
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

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])