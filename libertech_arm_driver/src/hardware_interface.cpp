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

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "libertech_arm_driver/hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("LibertechArmInterfaceHardware");
}

namespace libertech_arm_driver
{
LibertechArmInterfaceHardware::LibertechArmInterfaceHardware()
  : joint_based_controller_running_(false), twist_controller_running_(false), first_pass_(true)
{
  // RCLCPP_INFO(LOGGER, "Setting severity threshold to DEBUG");
  // auto ret = rcutils_logging_set_logger_level(LOGGER.get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
  // if (ret != RCUTILS_RET_OK)
  // {
  //   RCLCPP_ERROR(LOGGER, "Error setting severity: %s", rcutils_get_error_string().str);
  //   rcutils_reset_error();
  // }
}

hardware_interface::CallbackReturn LibertechArmInterfaceHardware::on_init(const hardware_interface::HardwareInfo& info)
{
  RCLCPP_INFO(LOGGER, "Configuring Hardware Interface");
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  info_ = info;
  // The robot's param.
  std::string arm_type = info_.hardware_parameters["arm_type"];
  if (arm_type.empty())
  {
    RCLCPP_ERROR(LOGGER, "Arm type is empty!");
    return CallbackReturn::ERROR;
  }
  else
  {
    RCLCPP_INFO(LOGGER, "Arm type is '%s'", arm_type.c_str());
  }

  // // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  // hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  // hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  // hw_slowdown_ = stod(info_.hardware_parameters["example_param_hw_slowdown"]);
  // // END: This part here is for exemplary purposes - Please do not copy to your production code
  // hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  // hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  arm_positions_.resize(actuator_count_, std::numeric_limits<double>::quiet_NaN());
  arm_velocities_.resize(actuator_count_, std::numeric_limits<double>::quiet_NaN());
  arm_efforts_.resize(actuator_count_, std::numeric_limits<double>::quiet_NaN());
  arm_commands_positions_.resize(actuator_count_, std::numeric_limits<double>::quiet_NaN());
  arm_commands_velocities_.resize(actuator_count_, std::numeric_limits<double>::quiet_NaN());
  arm_commands_efforts_.resize(actuator_count_, std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // if (joint.command_interfaces.size() != 1)
    // {
    //   RCLCPP_FATAL(LOGGER, "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
    //                joint.command_interfaces.size());
    //   return CallbackReturn::ERROR;
    // }

    if (!(joint.command_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
          joint.command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
          joint.command_interfaces[0].name == hardware_interface::HW_IF_EFFORT))
    {
      RCLCPP_FATAL(
        LOGGER, "Joint '%s' has %s command interface. Expected %s, %s, or %s.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION,
        hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_EFFORT);
      return CallbackReturn::ERROR;
    }

    // if (joint.state_interfaces.size() != 1)
    // {
    //   RCLCPP_FATAL(LOGGER,
    //                "Joint '%s' has %zu state interface. 7 expected.", joint.name.c_str(),
    //                joint.state_interfaces.size());
    //   return CallbackReturn::ERROR;
    // }

    if (!(joint.state_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
          joint.state_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
          joint.state_interfaces[0].name == hardware_interface::HW_IF_EFFORT))
    {
      RCLCPP_FATAL(
        LOGGER, "Joint '%s' has %s state interface. Expected %s, %s, or %s.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION,
        hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_EFFORT);
      return CallbackReturn::ERROR;
    }
  }

  // // TODO：7自由度或14自由度的初始化，当前默认14 ????
  // for (const hardware_interface::ComponentInfo& joint : info_.joints)
  // {
  //   // has exactly one command interface on each joint
  //   if (joint.command_interfaces.size() != 7)
  //   {
  //     RCLCPP_FATAL(rclcpp::get_logger("LibertechArmInterfaceHardware"),
  //                  "Joint '%s' has %zu command interfaces found. 7 expected.", joint.name.c_str(),
  //                  joint.command_interfaces.size());
  //     return hardware_interface::CallbackReturn::ERROR;
  //   }

  //   if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
  //   {
  //     RCLCPP_FATAL(rclcpp::get_logger("LibertechArmInterfaceHardware"),
  //                  "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
  //                  joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
  //     return hardware_interface::CallbackReturn::ERROR;
  //   }

  //   // 假设只反馈位置
  //   if (joint.state_interfaces.size() != 7)
  //   {
  //     RCLCPP_FATAL(rclcpp::get_logger("LibertechArmInterfaceHardware"),
  //                  "Joint '%s' has %zu state interface. 7 expected.", joint.name.c_str(),
  //                  joint.state_interfaces.size());
  //     return hardware_interface::CallbackReturn::ERROR;
  //   }

  //   if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
  //   {
  //     RCLCPP_FATAL(rclcpp::get_logger("LibertechArmInterfaceHardware"),
  //                  "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
  //                  joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
  //     return hardware_interface::CallbackReturn::ERROR;
  //   }
  // }

  // if (arm_type) {

  // }
  // 初始化
  armSDK_ = new ArmSDK(0);
  armSDK_->init();
  armSDK_->runSendReceiveThread();  // 放在这里吗

  RCLCPP_INFO(LOGGER, "Hardware Interface successfully configured");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
LibertechArmInterfaceHardware::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  // // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  // RCLCPP_INFO(
  //   rclcpp::get_logger("LibertechArmInterfaceHardware"), "Configuring ...please wait...");

  // for (int i = 0; i < hw_start_sec_; i++)
  // {
  //   rclcpp::sleep_for(std::chrono::seconds(1));
  //   RCLCPP_INFO(
  //     rclcpp::get_logger("LibertechArmInterfaceHardware"), "%.1f seconds left...",
  //     hw_start_sec_ - i);
  // }
  // // END: This part here is for exemplary purposes - Please do not copy to your production code

  // // reset values always when configuring hardware
  // for (uint i = 0; i < hw_states_.size(); i++)
  // {
  //   hw_states_[i] = 0;
  //   hw_commands_[i] = 0;
  // }

  // RCLCPP_INFO(rclcpp::get_logger("LibertechArmInterfaceHardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> LibertechArmInterfaceHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &arm_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &arm_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &arm_efforts_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> LibertechArmInterfaceHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &arm_commands_positions_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &arm_commands_velocities_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &arm_commands_efforts_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn
LibertechArmInterfaceHardware::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("LibertechArmInterfaceHardware"), "Activating ...please wait...");

  // 设置模式 初始值

  RCLCPP_INFO(rclcpp::get_logger("LibertechArmInterfaceHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
LibertechArmInterfaceHardware::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("LibertechArmInterfaceHardware"), "Deactivating ...please wait...");

  armSDK_->pause();

  RCLCPP_INFO(rclcpp::get_logger("LibertechArmInterfaceHardware"), "Successfully deactivated!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type LibertechArmInterfaceHardware::read(const rclcpp::Time& /*time*/,
                                                                    const rclcpp::Duration& /*period*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  // RCLCPP_INFO(rclcpp::get_logger("LibertechArmInterfaceHardware"), "Reading...");

  // for (uint i = 0; i < hw_states_.size(); i++)
  // {
  //   // Simulate RRBot's movement
  //   hw_states_[i] = hw_states_[i] + (hw_commands_[i] - hw_states_[i]) / hw_slowdown_;
  //   RCLCPP_INFO(
  //     rclcpp::get_logger("LibertechArmInterfaceHardware"), "Got state %.5f for joint %d!",
  //     hw_states_[i], i);
  // }
  // RCLCPP_INFO(rclcpp::get_logger("LibertechArmInterfaceHardware"), "Joints successfully read!");
  // // END: This part here is for exemplary purposes - Please do not copy to your production code

  Vec7 pos, vel, tau;
  armSDK_->getQ(pos);
  armSDK_->getQd(vel);
  armSDK_->getTau(tau);

  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    // read torque
    arm_efforts_[i] = tau[i];  // N*m
    // read velocity
    arm_velocities_[i] = vel[i];  // rad/sec
    // read position
    arm_positions_[i] = pos[i];  // rad
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type LibertechArmInterfaceHardware::write(const rclcpp::Time& /*time*/,
                                                                     const rclcpp::Duration& /*period*/)
{
  // // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  // RCLCPP_INFO(rclcpp::get_logger("LibertechArmInterfaceHardware"), "Writing...");

  // for (uint i = 0; i < hw_commands_.size(); i++)
  // {
  //   // Simulate sending commands to the hardware
  //   RCLCPP_INFO(
  //     rclcpp::get_logger("LibertechArmInterfaceHardware"), "Got command %.5f for joint %d!",
  //     hw_commands_[i], i);
  // }
  // RCLCPP_INFO(
  //   rclcpp::get_logger("LibertechArmInterfaceHardware"), "Joints successfully written!");
  // // END: This part here is for exemplary purposes - Please do not copy to your production code

  // if (stop_joint_based_controller_)
  // {
  //   joint_based_controller_running_ = false;
  //   arm_commands_positions_ = arm_positions_;
  //   arm_commands_velocities_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  // }
  // if (stop_twist_controller_)
  // {
  //   twist_controller_running_ = false;
  //   twist_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  // }

  // if (start_joint_based_controller_)
  // {
  //   twist_controller_running_ = false;
  //   arm_commands_positions_ = arm_positions_;
  //   arm_commands_velocities_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  //   joint_based_controller_running_ = true;
  //   // refresh feedback
  //   feedback_ = base_cyclic_.RefreshFeedback();
  // }
  Vec7 pos;
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    pos[i] = arm_commands_positions_[i];  // rad
  }

  armSDK_->setQ(pos);

  return hardware_interface::return_type::OK;
}

}  // namespace libertech_arm_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(libertech_arm_driver::LibertechArmInterfaceHardware, hardware_interface::SystemInterface)
