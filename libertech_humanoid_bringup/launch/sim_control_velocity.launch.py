# Copyright (c) 2021 PickNik, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Marq Rasmussen

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
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
    # Initialize Arguments
    use_hand = LaunchConfiguration("use_hand")
    use_gripper = LaunchConfiguration("use_gripper")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    sim_ignition = LaunchConfiguration("sim_ignition")
    sim_ignition_world = LaunchConfiguration("sim_ignition_world")
    use_camera = LaunchConfiguration("use_camera")
    camera_width = LaunchConfiguration("camera_width")
    camera_height = LaunchConfiguration("camera_height")
    camera_fps = LaunchConfiguration("camera_fps")
    
    # General arguments
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")

    robot_controllers = PathJoinSubstitution(
        # https://answers.ros.org/question/397123/how-to-access-the-runtime-value-of-a-launchconfiguration-instance-within-custom-launch-code-injected-via-an-opaquefunction-in-ros2/
        [
            FindPackageShare(description_package),
            "config",
            controllers_file,
        ]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "view_robot.rviz"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ",
            "prefix:=",
            prefix,
            " ",
            "use_hand:=",
            use_hand,
            " ",
            "use_gripper:=",
            use_gripper,
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
            " ",
            "sim_ignition:=",
            sim_ignition,
            " ",
            "use_camera:=",
            use_camera,
            " ",
            "camera_width:=",
            camera_width,
            " ",
            "camera_height:=",
            camera_height,
            " ",
            "camera_fps:=",
            camera_fps,
            " ",
            "simulation_controllers:=",
            robot_controllers,
        ]
    )
    robot_description = {"robot_description": robot_description_content.perform(context)}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            robot_description,
            {"use_sim_time": use_sim_time},
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(launch_rviz),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    arm_right_vel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_right_velocity_controller", "-c", "/controller_manager"],
    )
    
    arm_left_vel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_left_velocity_controller", "-c", "/controller_manager"],
    )
    
    movement_traj_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["movement_trajectory_controller", "-c", "/controller_manager"],
    )
    
    head_vel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["head_velocity_controller", "-c", "/controller_manager"],
    )

    
    hand_right_traj_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["hand_right_trajectory_controller", "-c", "/controller_manager"],
        condition=IfCondition(use_hand),
    )
    
    hand_left_traj_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["hand_left_trajectory_controller", "-c", "/controller_manager"],
        condition=IfCondition(use_hand),
    )
        
    # Bridge
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
        output="screen",
    )

    initial_head_joint_config = PathJoinSubstitution(
        [
            FindPackageShare(description_package),
            "config",
            "initial_positions_head.yaml",
        ]
    )
    
    ignition_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            # "-string",
            # robot_description_content,
            '-topic', 
            'robot_description',
            "-name",
            "arm",
            "-allow_renaming",
            "true",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "1.0",
            "-R",
            "0.0",
            "-P",
            "0.0",
            "-Y",
            "0.0",
        ],
        condition=IfCondition(sim_ignition),
    )
    
    ignition_world = PathJoinSubstitution(
        [
            FindPackageShare("libertech_humanoid_bringup"),
            "world",
            sim_ignition_world,
        ]
    )

    ignition_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        # launch_arguments={"ign_args": " -r -v 3 empty.sdf"}.items(),
        launch_arguments={"ign_args": f" -r -v 1 {ignition_world.perform(context)}"}.items(),
        condition=IfCondition(sim_ignition),
    )

    # Bridge
    # gazebo_bridge = Node(
    #     package="ros_gz_bridge",
    #     executable="parameter_bridge",
    #     parameters=[{"use_sim_time": use_sim_time}],
    #     arguments=[
    #         "/wrist_mounted_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image",
    #         "/wrist_mounted_camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image",
    #         "/wrist_mounted_camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
    #         "/wrist_mounted_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
    #     ],
    #     output="screen",
    # )

    nodes_to_start = [
        bridge,
        robot_state_publisher_node,
        ignition_launch_description,
        ignition_spawn_entity,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=ignition_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=ignition_spawn_entity,
                on_exit=[arm_right_vel_controller_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=ignition_spawn_entity,
                on_exit=[arm_left_vel_controller_spawner],
            )
        ),
        
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=ignition_spawn_entity,
                on_exit=[movement_traj_controller_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=ignition_spawn_entity,
                on_exit=[head_vel_controller_spawner],
            )
        ),
        
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=ignition_spawn_entity,
                on_exit=[hand_right_traj_controller_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=ignition_spawn_entity,
                on_exit=[hand_left_traj_controller_spawner],
            )
        ),
        delay_rviz_after_joint_state_broadcaster_spawner
        # gazebo_bridge,
    ]

    return nodes_to_start

def generate_launch_description():
    declared_arguments = []
    # Simulation specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_hand",
            default_value="false",
            description="Use hand.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_gripper",
            default_value="false",
            description="Use gripper.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_ignition",
            default_value="true",
            description="Use Gazebo Ignition for simulation",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_ignition_world",
            default_value="world_empty.sdf", # world_ycb world_box world_empty
            description="Gazebo Ignition world",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_camera",
            default_value="true",
            description="Use head camera.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "camera_width",
            default_value="640",
            description="camera width",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "camera_height",
            default_value="480",
            description="camera height",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "camera_fps",
            default_value="30",
            description="camera fps",
        )
    )
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="ros2_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="libertech_humanoid_description",
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="humanoid_fixed.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_controller",
            default_value="arm_right_trajectory_controller",
            description="Robot controller to start.",
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
        DeclareLaunchArgument("launch_rviz", default_value="false", description="Launch RViz?")
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
