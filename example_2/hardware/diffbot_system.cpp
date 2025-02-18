// diffbot_system.cpp
// Copyright 2021 ros2_control Development Team
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

#include "ros2_control_demo_example_2/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace ros2_control_demo_example_2
{

hardware_interface::CallbackReturn DiffBotSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  // Call base class initialization.
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  // If rclcpp hasn't been initialized, do it here.
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  
  // Create our own node if one hasn't been provided.
  // (This is not typical; normally the node is passed in externally.)
  if (!node_) {
    node_ = rclcpp::Node::make_shared("diffbot_internal_node");
  }

  // Set up logger and clock.
  logger_ = std::make_shared<rclcpp::Logger>(rclcpp::get_logger("DiffBotSystemHardware"));
  clock_ = std::make_shared<rclcpp::Clock>(rclcpp::Clock());

  // Retrieve hardware parameters.
  hw_start_sec_ = hardware_interface::stod(info.hardware_parameters.at("example_param_hw_start_duration_sec"));
  hw_stop_sec_  = hardware_interface::stod(info.hardware_parameters.at("example_param_hw_stop_duration_sec"));

  hw_positions_.resize(info.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info.joints.size(), std::numeric_limits<double>::quiet_NaN());

  // Use our node pointer to create the publisher.
  json_publisher_ = node_->create_publisher<std_msgs::msg::String>("publish_vel", 10);

  // Validate joint interfaces (simplified checks).
  for (const auto & joint : info.joints)
  {
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(get_logger(), "Joint '%s' has %zu command interfaces. 1 expected.",
                   joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(get_logger(), "Joint '%s' has '%s' command interface. '%s' expected.",
                   joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
                   hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(get_logger(), "Joint '%s' has %zu state interfaces. 2 expected.",
                   joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(get_logger(), "Joint '%s' has '%s' as first state interface. '%s' expected.",
                   joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
                   hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(get_logger(), "Joint '%s' has '%s' as second state interface. '%s' expected.",
                   joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
                   hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffBotSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffBotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]);
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Activating...");
  for (size_t i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_start_sec_ - i);
  }
  for (size_t i = 0; i < hw_positions_.size(); i++)
  {
    if (std::isnan(hw_positions_[i]))
    {
      hw_positions_[i] = 0;
      hw_velocities_[i] = 0;
      hw_commands_[i] = 0;
    }
  }
  RCLCPP_INFO(get_logger(), "Activated successfully!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating...");
  for (size_t i = 0; i < hw_stop_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_stop_sec_ - i);
  }
  RCLCPP_INFO(get_logger(), "Deactivated successfully!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffBotSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  std::stringstream ss;
  ss << "Reading states:";
  for (size_t i = 0; i < hw_velocities_.size(); i++)
  {
    hw_positions_[i] = hw_positions_[i] + period.seconds() * hw_velocities_[i];
    ss << std::fixed << std::setprecision(2) << std::endl
       << "\tposition " << hw_positions_[i] << " and velocity " << hw_velocities_[i]
       << " for '" << info_.joints[i].name << "'!";
  }
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DiffBotSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  nlohmann::json msg;
  std::stringstream ss;
  ss << "Writing commands:";
  for (size_t i = 0; i < hw_commands_.size(); i++)
  {
    msg[info_.joints[i].name] = hw_commands_[i];
    hw_velocities_[i] = hw_commands_[i];
    ss << std::fixed << std::setprecision(2) << std::endl
       << "\tcommand " << hw_commands_[i] << " for '" << info_.joints[i].name << "'!";
  }
  std_msgs::msg::String pub_msg;
  pub_msg.data = msg.dump();
  json_publisher_->publish(pub_msg);
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());
  return hardware_interface::return_type::OK;
}

}  // namespace ros2_control_demo_example_2

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_example_2::DiffBotSystemHardware, hardware_interface::SystemInterface)
