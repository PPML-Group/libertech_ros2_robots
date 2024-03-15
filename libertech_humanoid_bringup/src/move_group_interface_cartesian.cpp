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
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// All source files that use ROS logging should define a file-specific
// static const rclcpp::Logger named LOGGER, located at the top of the file
// and inside the namespace with the narrowest scope (if there is one)
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

geometry_msgs::msg::Pose interpolatePose(const geometry_msgs::msg::Pose& start, const geometry_msgs::msg::Pose& end, double fraction) {
    geometry_msgs::msg::Pose middle;
    middle.position.x = start.position.x + fraction * (end.position.x - start.position.x);
    middle.position.y = start.position.y + fraction * (end.position.y - start.position.y);
    middle.position.z = start.position.z + fraction * (end.position.z - start.position.z);
   
    // 方向（四元数）插值
    tf2::Quaternion start_quat, end_quat, middle_quat;
    // 将geometry_msgs的四元数转换为tf2的四元数
    tf2::fromMsg(start.orientation, start_quat);
    tf2::fromMsg(end.orientation, end_quat);
    // 使用Slerp进行四元数插值
    middle_quat = start_quat.slerp(end_quat, fraction);
    // 将插值后的四元数转换回geometry_msgs类型
    // middle.orientation = tf2::toMsg(middle_quat);

     // 假设方向保持不变
    middle.orientation = start.orientation;
    return middle;
}


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();
  static const std::string PLANNING_GROUP = "arm_right";
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
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

  // Start the demo
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  // .. _move_group_interface-planning-to-pose-goal:
  //
  // Planning to a Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the
  // end-effector.
  //  geometry_msgs::msg::Pose target_pose1;
  //   target_pose1.orientation.w = 0.56098;
  //   target_pose1.orientation.x = 0.43046;
  //   target_pose1.orientation.y = 0.56098;
  //   target_pose1.orientation.z = 0.43046;

  //   target_pose1.position.x = 0.40;
  //   target_pose1.position.y = -0.25;
  //   target_pose1.position.z = 0.0;

    // geometry_msgs::msg::Pose target_pose2;
    // target_pose2.orientation.w = 0.748178;
    // target_pose2.orientation.x = 0.607012;
    // target_pose2.orientation.y = 0.231413;
    // target_pose2.orientation.z = 0.30162;

    // target_pose2.position.x = 0.332245;
    // target_pose2.position.y = -0.6139;
    // target_pose2.position.z = 0.099;
  //  move_group.setPoseTarget(target_pose1);

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  // moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  // bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  // RCLCPP_INFO(LOGGER, "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // // Visualizing plans
  // // ^^^^^^^^^^^^^^^^^
  // // We can also visualize the plan as a line with markers in RViz.
  // RCLCPP_INFO(LOGGER, "Visualizing plan 1 as trajectory line");
  // visual_tools.publishAxisLabeled(target_pose1, "pose1");
  // visual_tools.publishText(text_pose, "Pose_Goal", rvt::WHITE, rvt::XLARGE);
  // visual_tools.publishTrajectoryLine(my_plan.trajectory, joint_model_group);
  // visual_tools.trigger();
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // Moving to a pose goal
  // ^^^^^^^^^^^^^^^^^^^^^
  //
  // Moving to a pose goal is similar to the step above
  // except we now use the ``move()`` function. Note that
  // the pose goal we had set earlier is still active
  // and so the robot will try to move to that goal. We will
  // not use that function in this tutorial since it is
  // a blocking function and requires a controller to be active
  // and report success on execution of a trajectory.

  /* Uncomment below line when working with a real robot */
  //  move_group.move(); 

  // Planning to a joint-space goal
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Let's set a joint space goal and move towards it.  This will replace the
  // pose target we set above.
  //
  // To start, we'll create an pointer that references the current robot's state.
  // RobotState is the object that contains all the current position/velocity/acceleration data.
  // moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);
  
  // Next get the current set of joint values for the group.
  // std::vector<double> joint_group_positions;
  // current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // Now, let's modify one of the joints, plan to the new joint space goal, and visualize the plan.
  // joint_group_positions[0] = 0.0;  // radians
  // joint_group_positions[1] = 0.0;
  // joint_group_positions[2] = 0.0;
  // joint_group_positions[3] = 0.0;
  // joint_group_positions[4] = 0.0;
  // joint_group_positions[5] = 0.0;
  // joint_group_positions[6] = 0.0;
  // bool within_bounds = move_group.setJointValueTarget(joint_group_positions);
  // if (!within_bounds)
  // {
  //   RCLCPP_WARN(LOGGER, "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
  // }

  // We lower the allowed maximum velocity and acceleration to 5% of their maximum.
  // The default values are 10% (0.1).
  // Set your preferred defaults in the joint_limits.yaml file of your robot's moveit_config
  // or set explicit factors in your code if you need your robot to move faster.
  // move_group.setMaxVelocityScalingFactor(0.05);
  // move_group.setMaxAccelerationScalingFactor(0.05);

  // success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  // RCLCPP_INFO(LOGGER, "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

  // move_group.move(); 
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
  geometry_msgs::msg::Pose target_pose2;
    target_pose2.orientation.w = 0.748178;
    target_pose2.orientation.x = 0.607012;
    target_pose2.orientation.y = 0.231413;
    target_pose2.orientation.z = 0.30162;

    target_pose2.position.x = 0.332245;
    target_pose2.position.y = -0.6139;
    target_pose2.position.z = 0.099;

    // target_pose2.orientation.w = 0.56098;
    // target_pose2.orientation.x = 0.43046;
    // target_pose2.orientation.y = 0.56098;
    // target_pose2.orientation.z = 0.43046;

    // target_pose2.position.x = 0.40;
    // target_pose2.position.y = -0.25;
    // target_pose2.position.z = 0.0;

    // target_pose2.orientation.w = 0.56098;
    // target_pose2.orientation.x = 0.43046;
    // target_pose2.orientation.y = 0.56098;
    // target_pose2.orientation.z = 0.43046;

    // target_pose2.position.x = 0.040;
    // target_pose2.position.y = -0.66;
    // target_pose2.position.z = 0.46;

    // target_pose2.position.x = 0.40;
    // target_pose2.position.y = -0.25;
    // target_pose2.position.z = 0.0;

    // target_pose2.orientation.x = -0.15106;
    // target_pose2.orientation.y = 0.65388;
    // target_pose2.orientation.z = 0.1191;
    // target_pose2.orientation.w = 0.73173;

    // target_pose2.position.x = 0.6299;
    // target_pose2.position.y = -0.145;
    // target_pose2.position.z = 0.427;


    // 在这里插入生成中间位姿点的代码
  std::vector<geometry_msgs::msg::Pose> waypoints;
  const int num_intermediate_points = 100; // 包括起始点，但不包括最终目标点
  // 获取当前的起始位姿
  geometry_msgs::msg::Pose start_pose = move_group.getCurrentPose().pose;
  // // 生成中间位姿点
  // for (int i = 1; i <= num_intermediate_points; i++) {
  //   double fraction = static_cast<double>(i) / (num_intermediate_points + 1);
  //   geometry_msgs::msg::Pose intermediate_pose = interpolatePose(start_pose, target_pose2, fraction);
  //   waypoints.push_back(intermediate_pose);
  // }

  // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  // moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);
  // moveit::core::RobotState start_state(*move_group.getCurrentState());
  // move_group.setStartState(start_state);
  // move_group.setPoseTarget(target_pose2);
  // move_group.setPlanningTime(30.0);
  // // std::vector<geometry_msgs::msg::Pose> waypoints;
  // // target_pose1.position.x += 0.1;
  // // waypoints.push_back(target_pose2);  // down
  // // target_pose2.position.x += 0.1;
  // waypoints.push_back(target_pose2);  // down
  // // target_pose2.position.y -= 0.1;
  // // waypoints.push_back(target_pose2);  // down
  // // target_pose2.position.z -= 0.05;
  // // waypoints.push_back(target_pose2);  

  waypoints.push_back(start_pose);
  waypoints.push_back(target_pose2);
 
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshold = 0;
  const double eef_step = 0.005;
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  RCLCPP_INFO(LOGGER, "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

   move_group.execute(trajectory); 

  rclcpp::shutdown();
  return 0;
}