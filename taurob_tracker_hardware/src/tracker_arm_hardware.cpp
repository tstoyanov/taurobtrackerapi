// Copyright (c) 2025 taurob GmbH. All rights reserved.
// 3-clause BSD; see COPYING.

#include "taurob_tracker_hardware/tracker_arm_hardware.hpp"

#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace taurob_tracker_hardware
{
namespace
{
rclcpp::Logger arm_logger()
{
  return rclcpp::get_logger("TrackerArmHardware");
}
}  // namespace

hardware_interface::CallbackReturn TrackerArmHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.empty())
  {
    RCLCPP_ERROR(arm_logger(), "no arm joints configured");
    return hardware_interface::CallbackReturn::ERROR;
  }

  Arm_config config;

  for (const auto & joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 1 ||
      joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_ERROR(
        arm_logger(), "joint '%s' must have exactly one position command interface",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces.size() != 1 ||
      joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_ERROR(
        arm_logger(), "joint '%s' must have exactly one position state interface",
        joint.name.c_str());
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

    config.Add_segment(joint.name, ip, port, tolerance_mrad);
    joint_names_.push_back(joint.name);
    joint_offsets_.push_back(offset);
  }

  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_commands_.resize(info_.joints.size(), 0.0);

  std::string watchdog = "true";
  if (info_.hardware_parameters.count("watchdog_enabled") > 0)
  {
    watchdog = info_.hardware_parameters.at("watchdog_enabled");
  }
  watchdog_enabled_ = (watchdog != "false");
  if (watchdog_enabled_)
  {
    watchdog_gate_ = std::make_shared<WatchdogGate>();
  }

  arm_ = std::make_unique<Arm>(config);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TrackerArmHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  arm_->Run();
  RCLCPP_INFO(arm_logger(), "arm activated with %zu segments", joint_names_.size());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TrackerArmHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  arm_->Stop();
  RCLCPP_INFO(arm_logger(), "arm deactivated");
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
  arm_->Feed_watchdog();
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type TrackerArmHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  ARM_SEGMENT_STATE command_state = POSITION_CONTROL;
  if (watchdog_enabled_ && !(watchdog_gate_ && watchdog_gate_->is_ok()))
  {
    // do not send motion commands while the safety watchdog is not fed
    command_state = STOP;
  }

  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    ARM_SEGMENT_COMMAND command;
    command.state = command_state;
    command.set_position = static_cast<float>(hw_commands_[i] + joint_offsets_[i]);
    command.max_speed = 0.0f;
    command.set_speed = 0.0f;
    command.reset_friction_clutch = false;
    command.control_aux_port = false;
    command.enable_five_volt = false;
    command.enable_twelve_volt = false;
    command.enable_dig_out_1 = false;
    command.enable_dig_out_2 = false;
    command.reset_u_ecu = false;
    arm_->Set(static_cast<int>(i), command);
  }
  return hardware_interface::return_type::OK;
}

}  // namespace taurob_tracker_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  taurob_tracker_hardware::TrackerArmHardware, hardware_interface::SystemInterface)
