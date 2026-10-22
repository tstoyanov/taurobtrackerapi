// Copyright (c) 2025 taurob GmbH. All rights reserved.
// 3-clause BSD; see COPYING.

#include "taurob_tracker_hardware/tracker_flipper_hardware.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace taurob_tracker_hardware
{
namespace { rclcpp::Logger trackerflipperhardware_logger() { return rclcpp::get_logger("TrackerFlipperHardware"); } }


hardware_interface::CallbackReturn TrackerFlipperHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != 1)
  {
    RCLCPP_ERROR(trackerflipperhardware_logger(), "expected exactly 1 flipper joint, got %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }
  const auto & joint = info_.joints[0];
  if (joint.command_interfaces.size() != 1 ||
    joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
  {
    RCLCPP_ERROR(
      trackerflipperhardware_logger(), "joint '%s' must have exactly one position command interface", joint.name.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  flipper_ip_ = "10.0.0.5";
  flipper_port_ = 10002;
  if (info_.hardware_parameters.count("flipper_ip") > 0)
  {
    flipper_ip_ = info_.hardware_parameters.at("flipper_ip");
  }
  if (info_.hardware_parameters.count("flipper_port") > 0)
  {
    flipper_port_ = std::stoi(info_.hardware_parameters.at("flipper_port"));
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

hardware_interface::CallbackReturn TrackerFlipperHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  flipper_ = std::make_unique<Flipper>(flipper_ip_, flipper_port_);
  flipper_->Run();
  RCLCPP_INFO(trackerflipperhardware_logger(), "flipper activated at %s:%d", flipper_ip_.c_str(), flipper_port_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TrackerFlipperHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (flipper_)
  {
    flipper_->Stop();
    flipper_.reset();
  }
  RCLCPP_INFO(trackerflipperhardware_logger(), "flipper deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TrackerFlipperHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (flipper_)
  {
    flipper_->Stop();
    flipper_.reset();
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> TrackerFlipperHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(
    info_.joints[0].name, hardware_interface::HW_IF_POSITION, &hw_position_);
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
TrackerFlipperHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back(
    info_.joints[0].name, hardware_interface::HW_IF_POSITION, &hw_command_);
  return command_interfaces;
}

hardware_interface::return_type TrackerFlipperHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!flipper_)
  {
    return hardware_interface::return_type::ERROR;
  }
  flipper_->Feed_watchdog();
  hw_position_ = flipper_->Get().angle;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type TrackerFlipperHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!flipper_)
  {
    return hardware_interface::return_type::ERROR;
  }
  FlipperCommand command;
  if (watchdog_enabled_ && !(watchdog_gate_ && watchdog_gate_->is_ok()))
  {
    // do not move while the safety watchdog is not fed; hold current angle
    command.angle = hw_position_;
  }
  else
  {
    command.angle = hw_command_;
  }
  flipper_->Set(command);
  return hardware_interface::return_type::OK;
}

}  // namespace taurob_tracker_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  taurob_tracker_hardware::TrackerFlipperHardware, hardware_interface::SystemInterface)
