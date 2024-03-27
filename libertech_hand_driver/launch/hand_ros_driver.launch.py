from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    TimerAction
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
    
    urdf_file_gen_name = 'hand_' + hand_type.perform(context) + '.xacro.urdf'
    urdf_file_gen = PathJoinSubstitution(
        [FindPackageShare("libertech_hand_description"), "urdf", f"{urdf_file_gen_name}"]
    )
    
    with open(urdf_file_gen.perform(context), "w") as f:
        f.write(robot_description_content.perform(context))
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description]
    )
    
    hand_driver = Node(
        package='libertech_hand_driver',
        executable='hand_ros_driver.py',
        name='hand_ros_driver',
        parameters=[{'urdf_file': urdf_file_gen},
                    {'action_file': action_file},
                    {'ip': hand_ip},
                    {'sim': sim}],
        output='screen',
    )
    
    rviz_file = PathJoinSubstitution(
        [FindPackageShare("libertech_hand_description"), "rviz", "urdf.rviz"]
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_file],
    )
    
    nodes_to_start = [
        hand_driver,
        TimerAction(period=3.0, actions=[robot_state_publisher_node]),
        rviz_node,
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