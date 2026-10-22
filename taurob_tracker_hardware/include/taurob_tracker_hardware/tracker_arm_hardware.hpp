// Copyright (c) 2025 taurob GmbH. All rights reserved.
// 3-clause BSD; see COPYING.

#ifndef TAUROB_TRACKER_HARDWARE__TRACKER_ARM_HARDWARE_HPP_
#define TAUROB_TRACKER_HARDWARE__TRACKER_ARM_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "libtaurob_arm/libtaurob_arm.h"

namespace taurob_tracker_hardware
{

/// ros2_control ActuatorInterface-style SystemInterface for the taurob tracker arm.
///
/// Each joint of the arm is a position-controlled segment; per-joint offsets
/// (calibration) are given in rad via the joint's <param name="offset">.
class TrackerArmHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(TrackerArmHardware)

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
  std::vector<double> hw_positions_;
  std::vector<double> hw_commands_;
  std::vector<double> joint_offsets_;
  std::vector<std::string> joint_names_;

  std::unique_ptr<Arm> arm_;
};

}  // namespace taurob_tracker_hardware

#endif  // TAUROB_TRACKER_HARDWARE__TRACKER_ARM_HARDWARE_HPP_
