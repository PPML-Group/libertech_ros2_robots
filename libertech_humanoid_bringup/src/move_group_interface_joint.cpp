/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit_visual_tools/moveit_visual_tools.h>

// All source files that use ROS logging should define a file-specific
// static const rclcpp::Logger named LOGGER, located at the top of the file
// and inside the namespace with the narrowest scope (if there is one)
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

  // We spin up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // BEGIN_TUTORIAL
  //
  // Setup
  // ^^^^^
  //
  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the ``JointModelGroup``. Throughout MoveIt, the terms "planning group" and "joint model group"
  // are used interchangeably.
  static const std::string PLANNING_GROUP = "arm_right";

  // The
  // :moveit_codedir:`MoveGroupInterface<moveit_ros/planning_interface/move_group_interface/include/moveit/move_group_interface/move_group_interface.h>`
  // class can be easily set up using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

  // We will use the
  // :moveit_codedir:`PlanningSceneInterface<moveit_ros/planning_interface/planning_scene_interface/include/moveit/planning_scene_interface/planning_scene_interface.h>`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const moveit::core::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Visualization
  // ^^^^^^^^^^^^^
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools(move_group_node, "base", "move_group_tutorial",
                                                      move_group.getRobotModel());

  visual_tools.deleteAllMarkers();

  /* Remote control is an introspection tool that allows users to step through a high level script */
  /* via buttons and keyboard shortcuts in RViz */
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "MoveGroupInterface_Demo", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  RCLCPP_INFO(LOGGER, "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));
// 开始
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;


  // Planning to a joint-space goal
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Let's set a joint space goal and move towards it.  This will replace the
  // pose target we set above.
  //
  // To start, we'll create an pointer that references the current robot's state.
  // RobotState is the object that contains all the current position/velocity/acceleration data.
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);
  
  // Next get the current set of joint values for the group.
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // Now, let's modify one of the joints, plan to the new joint space goal, and visualize the plan.
  joint_group_positions[0] = 0.65;  // radians
  joint_group_positions[1] = 0.5;
  joint_group_positions[2] = 0.0;
  joint_group_positions[3] = 0.0;
  joint_group_positions[4] = -0.3;
  joint_group_positions[5] = -0.3;
  joint_group_positions[6] = 0.0;
  bool within_bounds = move_group.setJointValueTarget(joint_group_positions);
  if (!within_bounds)
  {
    RCLCPP_WARN(LOGGER, "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
  }

  // We lower the allowed maximum velocity and acceleration to 5% of their maximum.
  // The default values are 10% (0.1).
  // Set your preferred defaults in the joint_limits.yaml file of your robot's moveit_config
  // or set explicit factors in your code if you need your robot to move faster.
  move_group.setMaxVelocityScalingFactor(0.05);
  move_group.setMaxAccelerationScalingFactor(0.05);

  bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

  move_group.move(); 
  // Visualize the plan in RViz:
  // visual_tools.deleteAllMarkers();
  // visual_tools.publishText(text_pose, "Joint_Space_Goal", rvt::WHITE, rvt::XLARGE);
  // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  // visual_tools.trigger();
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // // Planning with Path Constraints
  // // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // //
  // // Path constraints can easily be specified for a link on the robot.
  // // Let's specify a path constraint and a pose goal for our group.
  // // First define the path constraint.
  // moveit_msgs::msg::OrientationConstraint ocm;
  // ocm.link_name = "hand_right_palm";
  // ocm.header.frame_id = "base";
  // ocm.orientation.w = 1.0;
  // ocm.absolute_x_axis_tolerance = 0.1;
  // ocm.absolute_y_axis_tolerance = 0.1;
  // ocm.absolute_z_axis_tolerance = 0.1;
  // ocm.weight = 1.0;

  // // Now, set it as the path constraint for the group.
  // moveit_msgs::msg::Constraints test_constraints;
  // test_constraints.orientation_constraints.push_back(ocm);
  // move_group.setPathConstraints(test_constraints);

  // // Enforce Planning in Joint Space
  // // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // //
  // // Depending on the planning problem MoveIt chooses between
  // // ``joint space`` and ``cartesian space`` for problem representation.
  // // Setting the group parameter ``enforce_joint_model_state_space:true`` in
  // // the ompl_planning.yaml file enforces the use of ``joint space`` for all plans.
  // //
  // // By default, planning requests with orientation path constraints
  // // are sampled in ``cartesian space`` so that invoking IK serves as a
  // // generative sampler.
  // //
  // // By enforcing ``joint space``, the planning process will use rejection
  // // sampling to find valid requests. Please note that this might
  // // increase planning time considerably.
  // //
  // // We will reuse the old goal that we had and plan to it.
  // // Note that this will only work if the current state already
  // // satisfies the path constraints. So we need to set the start
  // // state to a new pose.
  // moveit::core::RobotState start_state(*move_group.getCurrentState());
  // geometry_msgs::msg::Pose start_pose2;
  // start_pose2.orientation.w = 0;
  // start_pose2.position.x = 0.3;
  // start_pose2.position.y = 0.3;
  // start_pose2.position.z = 0.5;
  // start_state.setFromIK(joint_model_group, start_pose2);
  // move_group.setStartState(start_state);

  // // // Now, we will plan to the earlier pose target from the new
  // // // start state that we just created.
  // move_group.setPoseTarget(target_pose1);

  // // // Planning with constraints can be slow because every sample must call an inverse kinematics solver.
  // // // Let's increase the planning time from the default 5 seconds to be sure the planner has enough time to succeed.
  // move_group.setPlanningTime(10.0);

  // bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  // RCLCPP_INFO(LOGGER, "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");

  // // Visualize the plan in RViz:
  // // visual_tools.deleteAllMarkers();
  // // visual_tools.publishAxisLabeled(start_pose2, "start");
  // // visual_tools.publishAxisLabeled(target_pose1, "goal");
  // // visual_tools.publishText(text_pose, "Constrained_Goal", rvt::WHITE, rvt::XLARGE);
  // // visual_tools.publishTrajectoryLine(my_plan.trajectory, joint_model_group);
  // // visual_tools.trigger();
  // // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // // // When done with the path constraint, be sure to clear it.
  // move_group.clearPathConstraints();



  rclcpp::shutdown();
  return 0;
}