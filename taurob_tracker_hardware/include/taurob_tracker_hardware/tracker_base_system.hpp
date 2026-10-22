// Copyright (c) 2025 taurob GmbH. All rights reserved.
// 3-clause BSD; see COPYING.

#ifndef TAUROB_TRACKER_HARDWARE__TRACKER_BASE_SYSTEM_HPP_
#define TAUROB_TRACKER_HARDWARE__TRACKER_BASE_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "libtaurob_base/libtaurob_base.h"

#include "taurob_tracker_hardware/watchdog_gate.hpp"

namespace taurob_tracker_hardware
{

/// ros2_control SystemInterface for the taurob tracker base platform.
///
/// Exposes two wheel (track) joints for velocity commands and position/velocity
/// states for use with diff_drive_controller, plus GPIO state interfaces for
/// battery voltage, temperature, motor currents and remaining operation time.
///
/// All ECU communication is gated on the safety watchdog: write() only sends
/// motion commands to the ECU while watchdog_feed pulses arrive on the
/// watchdog_feed topic (published by taurob_watchdog).
class TrackerBaseSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(TrackerBaseSystem)

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
  // libtaurob delivers DRIVETRAIN_STATUS via a plain function-pointer callback
  // with no context; cache it statically (single-instance rule enforced by the
  // safety requirements anyway).
  static DRIVETRAIN_STATUS latest_status_;
  static void on_status_received(DRIVETRAIN_STATUS status) { latest_status_ = status; }

  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;

  // sensor / gpio states
  double hw_supply_voltage_ = 0.0;
  double hw_temperature_ = 0.0;
  double hw_left_motor_current_ = 0.0;
  double hw_right_motor_current_ = 0.0;
  double hw_remaining_optime_ = 0.0;

  double odom_distance_ = 0.0;
  double odom_angle_ = 0.0;
  rclcpp::Time last_odom_time_;

  // parameters
  std::string base_ip_;
  int base_port_ = 8080;
  int protocol_version_ = 2;
  double gear_ratio_ = 0.0;
  double wheel_diameter_ = 0.303;
  double track_width_ = 0.55;
  double turning_geometry_factor_ = 1.0;

  bool watchdog_enabled_ = true;
  std::shared_ptr<WatchdogGate> watchdog_gate_;

  std::unique_ptr<Taurob_base> base_;
};

}  // namespace taurob_tracker_hardware

#endif  // TAUROB_TRACKER_HARDWARE__TRACKER_BASE_SYSTEM_HPP_
