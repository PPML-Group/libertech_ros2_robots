#!/usr/bin/env python3
"""
A script to outline the fundamentals of the moveit_py motion planning API.
"""

import time

# generic ros libraries
import rclpy
from rclpy.logging import get_logger

# moveit python library
from moveit.core.robot_state import RobotState # type: ignore
from moveit.planning import ( # type: ignore
    MoveItPy,
    MultiPipelinePlanRequestParameters,
)


def plan_and_execute(
    erobot,
    planning_component,
    logger,
    single_plan_parameters=None,
    multi_plan_parameters=None,
    sleep_time=0.0,
):
    """Helper function to plan and execute a motion."""
    # plan to goal
    logger.info("Planning trajectory")
    if multi_plan_parameters is not None:
        plan_result = planning_component.plan(
            multi_plan_parameters=multi_plan_parameters
        )
    elif single_plan_parameters is not None:
        plan_result = planning_component.plan(
            single_plan_parameters=single_plan_parameters
        )
    else:
        plan_result = planning_component.plan()

    # execute the plan
    if plan_result:
        logger.info("Executing plan")
        robot_trajectory = plan_result.trajectory
        erobot.execute(robot_trajectory, controllers=[])
    else:
        logger.error("Planning failed")

    time.sleep(sleep_time)


def main():

    ###################################################################
    # MoveItPy Setup
    ###################################################################
    rclpy.init()
    logger = get_logger("moveit_py.pose_goal")

    # instantiate MoveItPy instance and get planning component
    libertech = MoveItPy(node_name="moveit_py")
    libertech_right_arm = libertech.get_planning_component("robot")
    logger.info("MoveItPy instance created")

    
    # Plan 3 - set goal state with PoseStamped message
    ###########################################################################

    # set plan start state to current state
    libertech_right_arm.set_start_state_to_current_state()

    # set pose goal with PoseStamped message
    from geometry_msgs.msg import PoseStamped

    pose_goal = PoseStamped()
    pose_goal.header.frame_id = "base"
    pose_goal.pose.orientation.w = 1.0
    pose_goal.pose.position.x = 0.28
    pose_goal.pose.position.y = -0.2
    pose_goal.pose.position.z = 0.5
    libertech_right_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="hand_right_palm")

    # plan to goal
    plan_and_execute(libertech, libertech_right_arm, logger, sleep_time=3.0)

    

if __name__ == "__main__":
    main()