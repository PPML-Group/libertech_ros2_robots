from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    params = PathJoinSubstitution(
        [
            FindPackageShare("libertech_humanoid_vision"),
            "config",
            "grasp_cube_aruco_config.yaml",
        ]
    )

    return LaunchDescription(
        [
            Node(
                package="libertech_humanoid_vision",
                executable="grasp_cube_aruco.py",
                name="grasp_cube_aruco",
                parameters=[params],
                output={
                    "stdout": "screen",
                    "stderr": "screen",
                },
                emulate_tty=True,
            )
        ]
    )
