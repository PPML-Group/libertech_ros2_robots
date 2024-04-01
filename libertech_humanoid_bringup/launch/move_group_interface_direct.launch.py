from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    declared_arguments = []
    # Simulation specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_ignition",
            default_value="true",
            description="Use Ignition for simulation",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulated clock",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    )

    # Initialize Arguments
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    sim_ignition = LaunchConfiguration("sim_ignition")

    description_arguments = {
        "use_fake_hardware": "false",
        "sim_ignition": sim_ignition,
    }

    moveit_config = (
        MoveItConfigsBuilder("robot", package_name="libertech_humanoid_moveit_config")
        .robot_description(mappings=description_arguments)
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    # MoveGroupInterface demo executable
    move_group_demo = Node(
        name="move_group_interface_direct_dual",#move_group_interface_direct，move_group_interface_joint，move_group_interface_cartesian
        package="libertech_humanoid_bringup",
        executable="move_group_interface_direct_dual",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": use_sim_time}
        ],
    )
    
    # move_group_demo = Node(
    #     name="move_group_interface_cartesian",#move_group_interface_direct，move_group_interface_joint，move_group_interface_cartesian
    #     package="libertech_humanoid_bringup",
    #     executable="move_group_interface_cartesian",
    #     output="screen",
    #     parameters=[
    #         moveit_config.robot_description,
    #         moveit_config.robot_description_semantic,
    #         moveit_config.robot_description_kinematics,
    #         {"use_sim_time": use_sim_time}
    #     ],
    # )

    return LaunchDescription(declared_arguments + [move_group_demo])
