// Copyright 2020 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef LIBERTECH_ARM_DRIVER__HARDWARE_INTERFACE_HPP_
#define LIBERTECH_ARM_DRIVER__HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "libertech_arm_driver/visibility_control.h"
#include "libertech_arm_driver/ArmDriver.hpp"

namespace libertech_arm_driver
{
class LibertechArmInterfaceHardware : public hardware_interface::SystemInterface
{
public:
  LibertechArmInterfaceHardware();

  RCLCPP_SHARED_PTR_DEFINITIONS(LibertechArmInterfaceHardware)

  LIBERTECH_ARM_DRIVER_PUBLIC
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  LIBERTECH_ARM_DRIVER_PUBLIC
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  LIBERTECH_ARM_DRIVER_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  LIBERTECH_ARM_DRIVER_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  LIBERTECH_ARM_DRIVER_PUBLIC
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  LIBERTECH_ARM_DRIVER_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  LIBERTECH_ARM_DRIVER_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  LIBERTECH_ARM_DRIVER_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  // // Parameters for the RRBot simulation
  // double hw_start_sec_;
  // double hw_stop_sec_;
  // double hw_slowdown_;

  // // Store the command for the simulated robot
  // std::vector<double> hw_commands_;
  // std::vector<double> hw_states_;

  std::vector<double> arm_commands_positions_;
  std::vector<double> arm_commands_velocities_;
  std::vector<double> arm_commands_efforts_;
  std::vector<double> arm_positions_;
  std::vector<double> arm_velocities_;
  std::vector<double> arm_efforts_;

  // twist command interfaces
  std::vector<double> twist_commands_;

  // what controller is running
  bool joint_based_controller_running_;
  bool twist_controller_running_;

  // first pass flag
  bool first_pass_;

  int actuator_count_ = 14;
  ArmDriver* armDriver_;
};

}  // namespace libertech_arm_driver

#endif  // LIBERTECH_ARM_DRIVER__HARDWARE_INTERFACE_HPP_
