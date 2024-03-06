import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # moveit_config = (
    #     MoveItConfigsBuilder(
    #         robot_name="libertech", package_name="libertech_humanoid_moveit_config"
    #     )
    #     .to_moveit_configs()
    # )
    
    description_arguments = {
        "use_fake_hardware": "false",
        "sim_ignition": "true",
    }

    moveit_config = (
        MoveItConfigsBuilder("robot", package_name="libertech_humanoid_moveit_config")
        .robot_description(mappings=description_arguments)
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    example_file = DeclareLaunchArgument(
        "example_file",
        default_value="totural.py",
        description="Python API tutorial file name",
    )

    moveit_py_node = Node(
        name="moveit_py",
        package="libertech_humanoid_bringup",
        executable=LaunchConfiguration("example_file"),
        output="both",
        parameters=[moveit_config.to_dict()],
    )

    return LaunchDescription(
        [
            example_file,
            moveit_py_node,

        ]
    )