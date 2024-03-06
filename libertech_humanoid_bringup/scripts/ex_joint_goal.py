#!/usr/bin/env python3
"""
Example of moving to a joint configuration.
- ros2 run pymoveit2 ex_joint_goal.py --ros-args -p joint_positions:="[1.57, -1.57, 0.0, -1.57, 0.0, 1.57, 0.7854]"
"""

from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2

from typing import List

MOVE_GROUP_ARM: str = "arm_right"


def joint_names(prefix: str = "arm_right") -> List[str]:
    return [
        prefix + "shoulder_pitch_joint",
        prefix + "shoulder_roll_joint",
        prefix + "shoulder_yaw_joint",
        prefix + "elbow_pitch_joint",
        prefix + "elbow_roll_joint",
        prefix + "wrist_pitch_joint",
        prefix + "wrist_yaw_joint",
    ]


def base_link_name(prefix: str = "arm_right") -> str:
    return prefix + "link0"


def end_effector_name(prefix: str = "panda_") -> str:
    return prefix + "hand_tcp"


def gripper_joint_names(prefix: str = "panda_") -> List[str]:
    return [
        prefix + "finger_joint1",
        prefix + "finger_joint2",
    ]


def main():
    rclpy.init()

    # Create node for this example
    node = Node("ex_joint_goal")

    # Declare parameter for joint positions
    node.declare_parameter(
        "joint_positions",
        [
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ],
    )

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()
    
    # Create MoveIt 2 interface
    moveit2 = MoveIt2(
        node=node,
        joint_names=joint_names(),
        base_link_name="base",
        end_effector_name="hand_right_palm",
        group_name=MOVE_GROUP_ARM,
        callback_group=callback_group,
    )

    # Spin the node in background thread(s) and wait a bit for initialization
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    node.create_rate(1.0).sleep()

    # Scale down velocity and acceleration of joints (percentage of maximum)
    moveit2.max_velocity = 0.1
    moveit2.max_acceleration = 0.1

    # Get parameter
    joint_positions = (
        node.get_parameter("joint_positions").get_parameter_value().double_array_value
    )

    # Move to joint configuration
    node.get_logger().info(f"Moving to {{joint_positions: {list(joint_positions)}}}")
    moveit2.move_to_configuration(joint_positions)
    moveit2.wait_until_executed()

    rclpy.shutdown()
    executor_thread.join()
    exit(0)


if __name__ == "__main__":
    main()
