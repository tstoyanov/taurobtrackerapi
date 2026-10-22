// Copyright (c) 2025 taurob GmbH. All rights reserved.
// 3-clause BSD; see COPYING.

#include "taurob_tracker_hardware/tracker_base_system.hpp"
#include "taurob_tracker_hardware/watchdog_gate.hpp"

#include <chrono>
#include <cmath>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace taurob_tracker_hardware
{
namespace { rclcpp::Logger trackerbasesystem_logger() { return rclcpp::get_logger("TrackerBaseSystem"); } }


DRIVETRAIN_STATUS TrackerBaseSystem::latest_status_ = DRIVETRAIN_STATUS();

hardware_interface::CallbackReturn TrackerBaseSystem::on_init(
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
      trackerbasesystem_logger(), "expected exactly 2 wheel joints (left, right), got %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }
  for (const auto & joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 1 ||
      joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_ERROR(
        trackerbasesystem_logger(), "joint '%s' must have exactly one velocity command interface",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces.size() != 2 ||
      joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
      joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_ERROR(
        trackerbasesystem_logger(), "joint '%s' must have position and velocity state interfaces", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  hw_positions_.resize(2, 0.0);
  hw_velocities_.resize(2, 0.0);
  hw_commands_.resize(2, 0.0);

  auto get_param = [&](const std::string & name, std::string & value, const std::string & def) {
      value = def;
      if (info_.hardware_parameters.count(name) > 0)
      {
        value = info_.hardware_parameters.at(name);
      }
    };

  get_param("base_ip", base_ip_, "10.0.0.2");
  auto get_param_int = [&](const std::string & name, int & value, int def) {
      value = def;
      if (info_.hardware_parameters.count(name) > 0)
      {
        value = std::stoi(info_.hardware_parameters.at(name));
      }
    };
  get_param_int("base_port", base_port_, 8080);
  get_param_int("protocol_version", protocol_version_, 2);

  auto get_param_double = [&](const std::string & name, double & value, double def) {
      value = def;
      if (info_.hardware_parameters.count(name) > 0)
      {
        value = std::stod(info_.hardware_parameters.at(name));
      }
    };
  get_param_double("gear_ratio", gear_ratio_, 0.0);
  get_param_double("wheel_diameter", wheel_diameter_, 0.303);
  get_param_double("track_width", track_width_, 0.55);
  get_param_double("turning_geometry_factor", turning_geometry_factor_, 1.0);

  std::string watchdog;
  get_param("watchdog_enabled", watchdog, "true");
  watchdog_enabled_ = (watchdog != "false");
  if (watchdog_enabled_)
  {
    watchdog_gate_ = std::make_shared<WatchdogGate>();
  }

  RCLCPP_INFO(
    trackerbasesystem_logger(), "base ECU at %s:%d (protocol v%d)", base_ip_.c_str(), base_port_, protocol_version_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TrackerBaseSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  base_ = std::make_unique<Taurob_base>(
    base_ip_, base_port_, gear_ratio_, wheel_diameter_, track_width_, turning_geometry_factor_);
  base_->Set_on_received_callback(&TrackerBaseSystem::on_status_received);
  base_->Run();
  RCLCPP_INFO(trackerbasesystem_logger(), "base platform activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TrackerBaseSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (base_)
  {
    base_->Set_drive_command(0.0f, 0.0f);
    base_->Stop();
    base_.reset();
  }
  RCLCPP_INFO(trackerbasesystem_logger(), "base platform deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TrackerBaseSystem::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (base_)
  {
    base_->Set_drive_command(0.0f, 0.0f);
    base_->Stop();
    base_.reset();
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> TrackerBaseSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < 2; ++i)
  {
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
TrackerBaseSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < 2; ++i)
  {
    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]);
  }
  return command_interfaces;
}

hardware_interface::return_type TrackerBaseSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!base_)
  {
    return hardware_interface::return_type::ERROR;
  }

  base_->Feed_watchdog();

  // diff drive odometry from the drivetrain status
  // (distance_travelled_forward / angle_turned are deltas since the last update)
  const double distance = static_cast<double>(latest_status_.distance_travelled_forward);
  const double angle = static_cast<double>(latest_status_.angle_turned);

  hw_supply_voltage_ = static_cast<double>(latest_status_.voltage);
  hw_temperature_ = static_cast<double>(latest_status_.temperature);
  hw_left_motor_current_ = static_cast<double>(latest_status_.left_motor_current);
  hw_right_motor_current_ = static_cast<double>(latest_status_.right_motor_current);
  hw_remaining_optime_ = 0.0;  // computed from supply voltage by the broadcaster side

  const double left_delta = (distance - (angle * track_width_ / 2.0)) / 2.0;
  const double right_delta = (distance + (angle * track_width_ / 2.0)) / 2.0;
  hw_positions_[0] += left_delta / (wheel_diameter_ / 2.0);
  hw_positions_[1] += right_delta / (wheel_diameter_ / 2.0);
  // wheel angular velocities are not measured directly; expose track linear speeds as radial/s
  hw_velocities_[0] = static_cast<double>(latest_status_.driving_speed) / (wheel_diameter_ / 2.0);
  hw_velocities_[1] = static_cast<double>(latest_status_.driving_speed) / (wheel_diameter_ / 2.0);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type TrackerBaseSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!base_)
  {
    return hardware_interface::return_type::ERROR;
  }

  if (watchdog_enabled_ && !(watchdog_gate_ && watchdog_gate_->is_ok()))
  {
    // do not send motion commands while the safety watchdog is not fed
    base_->Set_drive_command(0.0f, 0.0f);
    return hardware_interface::return_type::OK;
  }

  // convert wheel angular velocities (rad/s) to track linear velocity (m/s) and
  // angular velocity (rad/s)
  const double left = hw_commands_[0] * (wheel_diameter_ / 2.0);
  const double right = hw_commands_[1] * (wheel_diameter_ / 2.0);
  const double linear = (left + right) / 2.0;
  const double angular = (right - left) / track_width_;
  base_->Set_drive_command(static_cast<float>(linear), static_cast<float>(angular));

  return hardware_interface::return_type::OK;
}

}  // namespace taurob_tracker_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  taurob_tracker_hardware::TrackerBaseSystem, hardware_interface::SystemInterface)
