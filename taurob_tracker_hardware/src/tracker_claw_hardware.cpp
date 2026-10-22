// Copyright (c) 2025 taurob GmbH. All rights reserved.
// 3-clause BSD; see COPYING.

#include "taurob_tracker_hardware/tracker_claw_hardware.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace taurob_tracker_hardware
{
namespace
{
rclcpp::Logger claw_logger()
{
  return rclcpp::get_logger("TrackerClawHardware");
}
}  // namespace

hardware_interface::CallbackReturn TrackerClawHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != 2)
  {
    RCLCPP_ERROR(
      claw_logger(), "expected exactly 2 claw joints (rotation, grip), got %zu",
      info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }
  for (const auto & joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 1 ||
      joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_ERROR(
        claw_logger(), "joint '%s' must have exactly one position command interface",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces.size() != 1 ||
      joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_ERROR(
        claw_logger(), "joint '%s' must have exactly one position state interface",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  rotation_joint_name_ = info_.joints[0].name;
  grip_joint_name_ = info_.joints[1].name;

  if (info_.sensors.size() > 1)
  {
    RCLCPP_ERROR(
      claw_logger(), "expected at most 1 force sensor, got %zu", info_.sensors.size());
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (info_.sensors.size() == 1)
  {
    force_sensor_name_ = info_.sensors[0].name;
  }

  claw_ip_ = "10.0.0.2";
  claw_port_ = 10003;
  if (info_.hardware_parameters.count("claw_ip") > 0)
  {
    claw_ip_ = info_.hardware_parameters.at("claw_ip");
  }
  if (info_.hardware_parameters.count("claw_port") > 0)
  {
    claw_port_ = std::stoi(info_.hardware_parameters.at("claw_port"));
  }

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

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TrackerClawHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  claw_ = std::make_unique<Claw>(claw_ip_, claw_port_, false);
  claw_->Run();
  RCLCPP_INFO(claw_logger(), "claw activated at %s:%d", claw_ip_.c_str(), claw_port_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TrackerClawHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (claw_)
  {
    claw_->Set_rotation(0.0f);
    claw_->Set_grip(0.0f);
    claw_->Stop();
    claw_.reset();
  }
  RCLCPP_INFO(claw_logger(), "claw deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TrackerClawHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (claw_)
  {
    claw_->Stop();
    claw_.reset();
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> TrackerClawHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(
    rotation_joint_name_, hardware_interface::HW_IF_POSITION, &hw_rotation_);
  state_interfaces.emplace_back(
    grip_joint_name_, hardware_interface::HW_IF_POSITION, &hw_grip_);
  state_interfaces.emplace_back(
    force_sensor_name_, hardware_interface::HW_IF_FORCE, &hw_force_);
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
TrackerClawHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back(
    rotation_joint_name_, hardware_interface::HW_IF_POSITION, &hw_rotation_command_);
  command_interfaces.emplace_back(
    grip_joint_name_, hardware_interface::HW_IF_POSITION, &hw_grip_command_);
  return command_interfaces;
}

hardware_interface::return_type TrackerClawHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!claw_)
  {
    return hardware_interface::return_type::ERROR;
  }
  claw_->Feed_watchdog();
  hw_rotation_ = claw_->Get_rotation();
  hw_grip_ = claw_->Get_grip();
  hw_force_ = claw_->Get_force();
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type TrackerClawHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!claw_)
  {
    return hardware_interface::return_type::ERROR;
  }
  if (watchdog_enabled_ && !(watchdog_gate_ && watchdog_gate_->is_ok()))
  {
    // do not actuate while the safety watchdog is not fed; hold current pose
    claw_->Set_rotation(hw_rotation_);
    claw_->Set_grip(hw_grip_);
    return hardware_interface::return_type::OK;
  }
  claw_->Set_rotation(static_cast<float>(hw_rotation_command_));
  claw_->Set_grip(static_cast<float>(hw_grip_command_));
  return hardware_interface::return_type::OK;
}

}  // namespace taurob_tracker_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  taurob_tracker_hardware::TrackerClawHardware, hardware_interface::SystemInterface)
