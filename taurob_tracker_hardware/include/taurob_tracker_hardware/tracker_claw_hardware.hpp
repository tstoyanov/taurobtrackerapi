// Copyright (c) 2025 taurob GmbH. All rights reserved.
// 3-clause BSD; see COPYING.

#ifndef TAUROB_TRACKER_HARDWARE__TRACKER_CLAW_HARDWARE_HPP_
#define TAUROB_TRACKER_HARDWARE__TRACKER_CLAW_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "libtaurob_claw/libtaurob_claw.h"

#include "taurob_tracker_hardware/watchdog_gate.hpp"

namespace taurob_tracker_hardware
{

/// ros2_control SystemInterface for the taurob claw/gripper.
///
/// Exposes two position-controlled joints (claw_rotation, claw_grip) plus the
/// measured grip force as a sensor state interface. Rotation is given in rad,
/// grip in rad (0 = open). Watchdog-gated like all tracker actuation.
class TrackerClawHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(TrackerClawHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  double hw_rotation_ = 0.0;
  double hw_grip_ = 0.0;
  double hw_force_ = 0.0;
  double hw_rotation_command_ = 0.0;
  double hw_grip_command_ = 0.0;

  std::string rotation_joint_name_ = "claw_rotation";
  std::string grip_joint_name_ = "claw_grip";
  std::string force_sensor_name_ = "claw_force_sensor";

  std::string claw_ip_;
  int claw_port_ = 10003;

  bool watchdog_enabled_ = true;
  std::shared_ptr<WatchdogGate> watchdog_gate_;

  std::unique_ptr<Claw> claw_;
};

}  // namespace taurob_tracker_hardware

#endif  // TAUROB_TRACKER_HARDWARE__TRACKER_CLAW_HARDWARE_HPP_
