# Copyright 2020 Open Source Robotics Foundation, Inc.
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

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
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
    robot_type = LaunchConfiguration("robot_type")
    description_package = LaunchConfiguration("description_package")
    
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "view_robot.rviz"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "robots", description_file]
            ),
            " ",
            "robot_ip:=xxx.yyy.zzz.www",
            " ",
            "name:=",
            robot_name,
            " ",
            "arm:=",
            robot_type,
            " ",
            "dof:=",
            dof,
            " ",
            "vision:=",
            vision,
            " ",
            "prefix:=",
            prefix,
            " ",
            "sim_gazebo:=",
            sim_gazebo,
            " ",
            "sim_ignition:=",
            sim_ignition,
            " ",
            "simulation_controllers:=",
            robot_controllers,
            " ",
            "gripper:=",
            "robotiq_2f_85",
            " ",
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
    
    
    nodes_to_start = [
        bridge,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        robot_traj_controller_spawner,
        robot_pos_controller_spawner,
        robot_hand_controller_spawner,
        gzserver,
        gzclient,
        gazebo_spawn_robot,
        ignition_launch_description,
        ignition_spawn_entity,
        gazebo_bridge,
    ]

    return nodes_to_start


def generate_launch_description():

    declared_arguments = []
    # Robot specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_type",
            description="Type/series of robot.",
            choices=["right", "left", "dual"],
            default_value="dual",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="libertech_arm_description",
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("libertech_arm_description"), "urdf", "kinova.urdf.xacro"]
            ),
            " ",
            "robot_ip:=xxx.yyy.zzz.www",
            " ",
            "name:=kinova",
            " ",
            "arm:=",
            robot_type,
            " ",
            "gripper:=",
            gripper,
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("kortex_description"), "rviz", "view_robot.rviz"]
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen",
    )

    nodes_to_start = [
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz,
    ]

    # return LaunchDescription(declared_arguments + nodes_to_start)


    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
