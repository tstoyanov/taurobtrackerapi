// Copyright (c) 2025 taurob GmbH. All rights reserved.
// 3-clause BSD; see COPYING.

#ifndef TAUROB_TRACKER_HARDWARE__TRACKER_FLIPPER_HARDWARE_HPP_
#define TAUROB_TRACKER_HARDWARE__TRACKER_FLIPPER_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "libtaurob_flipper/libtaurob_flipper.h"

namespace taurob_tracker_hardware
{

/// ros2_control SystemInterface for the taurob tracker front flipper.
///
/// The flipper is a single position-controlled joint; 0 rad is the neutral pose
/// (the pose the flipper had when the robot was switched on).
class TrackerFlipperHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(TrackerFlipperHardware)

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
  double hw_position_ = 0.0;
  double hw_command_ = 0.0;

  std::string flipper_ip_;
  int flipper_port_ = 10002;

  std::unique_ptr<Flipper> flipper_;
  rclcpp::Logger logger_;
};

}  // namespace taurob_tracker_hardware

#endif  // TAUROB_TRACKER_HARDWARE__TRACKER_FLIPPER_HARDWARE_HPP_
