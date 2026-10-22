// Copyright (c) 2025 taurob GmbH. All rights reserved.
// 3-clause BSD; see COPYING.

#include "taurob_tracker_hardware/tracker_arm_hardware.hpp"

#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace taurob_tracker_hardware
{

hardware_interface::CallbackReturn TrackerArmHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  logger_ = rclcpp::get_logger("TrackerArmHardware");

  if (info_.joints.empty())
  {
    RCLCPP_ERROR(logger_, "no arm joints configured");
    return hardware_interface::CallbackReturn::ERROR;
  }

  arm_ = std::make_unique<Arm>();

  size_t index = 0;
  for (const auto & joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 1 ||
      joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_ERROR(
        logger_, "joint '%s' must have exactly one position command interface",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces.size() != 1 ||
      joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_ERROR(
        logger_, "joint '%s' must have exactly one position state interface", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    std::string ip = "10.0.0.10";
    int port = 10001;
    int tolerance_mrad = 100;
    double offset = 0.0;

    if (joint.parameters.count("ip") > 0)
    {
      ip = joint.parameters.at("ip");
    }
    if (joint.parameters.count("port") > 0)
    {
      port = std::stoi(joint.parameters.at("port"));
    }
    if (joint.parameters.count("position_tolerance_mrad") > 0)
    {
      tolerance_mrad = std::stoi(joint.parameters.at("position_tolerance_mrad"));
    }
    if (joint.parameters.count("offset") > 0)
    {
      offset = std::stod(joint.parameters.at("offset"));
    }

    arm_->Add_segment(joint.name, ip, port, tolerance_mrad);
    joint_names_.push_back(joint.name);
    joint_offsets_.push_back(offset);
    joint_position_tolerance_mrad_.push_back(tolerance_mrad);
    ++index;
  }

  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_commands_.resize(info_.joints.size(), 0.0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TrackerArmHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  arm_->Run();
  RCLCPP_INFO(logger_, "arm activated with %zu segments", joint_names_.size());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TrackerArmHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  arm_->Stop();
  RCLCPP_INFO(logger_, "arm deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TrackerArmHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  arm_->Stop();
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> TrackerArmHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    state_interfaces.emplace_back(
      joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
TrackerArmHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    command_interfaces.emplace_back(
      joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_commands_[i]);
  }
  return command_interfaces;
}

hardware_interface::return_type TrackerArmHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    ARM_SEGMENT_STATUS status = arm_->Get(static_cast<int>(i));
    hw_positions_[i] = static_cast<double>(status.position) - joint_offsets_[i];
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type TrackerArmHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    ARM_SEGMENT_COMMAND command;
    command.mode = POSITION_CONTROL;
    command.set_position = static_cast<float>(hw_commands_[i] + joint_offsets_[i]);
    arm_->Set(static_cast<int>(i), command);
    arm_->Feed_watchdog();
  }
  return hardware_interface::return_type::OK;
}

}  // namespace taurob_tracker_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  taurob_tracker_hardware::TrackerArmHardware, hardware_interface::SystemInterface)
