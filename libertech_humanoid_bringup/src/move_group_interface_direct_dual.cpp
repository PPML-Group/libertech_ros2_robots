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

#include <atomic>
#include <thread>
#include <chrono>
#include <rclcpp/rclcpp.hpp>

// std::atomic<bool> start_signal(false);
// static const rclcpp::Logger LOGGER = rclcpp::get_logger("dual_arm_planning");

// void planAndMoveArm(const std::shared_ptr<rclcpp::Node>& node, const std::string& planning_group, const geometry_msgs::msg::Pose& target_pose) {
//     moveit::planning_interface::MoveGroupInterface move_group(node, planning_group);
//     move_group.setPoseTarget(target_pose);
    
//     // Planning
//     moveit::planning_interface::MoveGroupInterface::Plan my_plan;
//     bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

//     while (!start_signal.load()) {
//         std::this_thread::sleep_for(std::chrono::milliseconds(1));
//     }

//     if (success) {
//         move_group.move(); // Execute the planned path
//     } else {
//         RCLCPP_ERROR(node->get_logger(), "Planning for %s failed", planning_group.c_str());
//     }
// }

// int main(int argc, char** argv)
// {
//     rclcpp::init(argc, argv);
//     auto node = rclcpp::Node::make_shared("dual_arm_planning");

//     // 定义两个臂的目标位姿
//      geometry_msgs::msg::Pose target_pose_left; // 左臂目标位姿
//   geometry_msgs::msg::Pose target_pose_right; // 右臂目标位姿

//   target_pose_left.orientation.w = 0.560934;
//   target_pose_left.orientation.x = -0.430467;
//   target_pose_left.orientation.y = 0.561054;
//   target_pose_left.orientation.z = -0.430429;

//   target_pose_left.position.x = 0.41;
//   target_pose_left.position.y = 0.41;
//   target_pose_left.position.z = 0.24;

//   target_pose_right.orientation.w = 0.856;
//   target_pose_right.orientation.x = 0.1213;
//   target_pose_right.orientation.y = 0.5011;
//   target_pose_right.orientation.z = 0.0357;

//   target_pose_right.position.x = 0.515;
//   target_pose_right.position.y = -0.34;
//   target_pose_right.position.z = 0.63;
//     // 设置目标位姿的具体值...

//     // 分别为左臂和右臂创建并启动线程
//     // / 创建线程
    
//     std::thread thread_right(planAndMoveArm, node, "arm_right", target_pose_right);
//     std::thread thread_left(planAndMoveArm, node, "arm_left", target_pose_left);

//     start_signal.store(true);

//     // 等待线程完成
//     thread_left.join();
//     thread_right.join();

//     rclcpp::shutdown();
//     return 0;
// }


static const rclcpp::Logger LOGGER = rclcpp::get_logger("dual_arm_planning");
std::atomic<int> plan_completed_count(0);
std::atomic<bool> all_planned(false);
std::atomic<bool> start_moving(false);
constexpr int total_arms = 2; // 假设有两个臂需要规划

void planAndMoveArm(const std::shared_ptr<rclcpp::Node>& node, const std::string& planning_group, const geometry_msgs::msg::Pose& target_pose) {
    moveit::planning_interface::MoveGroupInterface move_group(node, planning_group);
    move_group.setPoseTarget(target_pose);
    
    // Planning
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
        plan_completed_count.fetch_add(1);
        RCLCPP_INFO(node->get_logger(), "Planning for %s succeeded", planning_group.c_str());
    } else {
        RCLCPP_ERROR(node->get_logger(), "Planning for %s failed", planning_group.c_str());
        return;
    }

    // 等待所有臂完成规划
    while (!all_planned.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        if (plan_completed_count.load() == total_arms) {
            all_planned.store(true);
        }
    }

    // 等待执行信号
    while (!start_moving.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // Execute the planned path
    move_group.move();
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("dual_arm_planning");

    // 定义两个臂的目标位姿
    geometry_msgs::msg::Pose target_pose_left; // 左臂目标位姿
    geometry_msgs::msg::Pose target_pose_right; // 右臂目标位姿
    // 初始化目标位姿...
    target_pose_left.orientation.w = 0.560934;
  target_pose_left.orientation.x = -0.430467;
  target_pose_left.orientation.y = 0.561054;
  target_pose_left.orientation.z = -0.430429;

  target_pose_left.position.x = 0.41;
  target_pose_left.position.y = 0.41;
  target_pose_left.position.z = 0.24;

  target_pose_right.orientation.w = 0.856;
  target_pose_right.orientation.x = 0.1213;
  target_pose_right.orientation.y = 0.5011;
  target_pose_right.orientation.z = 0.0357;

  target_pose_right.position.x = 0.515;
  target_pose_right.position.y = -0.34;
  target_pose_right.position.z = 0.63;

    // 分别为左臂和右臂创建并启动线程
    std::thread thread_left(planAndMoveArm, node, "arm_left", target_pose_left);
    std::thread thread_right(planAndMoveArm, node, "arm_right", target_pose_right);

    // 等待所有臂完成规划
    while (!all_planned.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // 发送执行信号
    start_moving.store(true);

    // 等待线程完成
    thread_left.join();
    thread_right.join();

    rclcpp::shutdown();
    return 0;
}