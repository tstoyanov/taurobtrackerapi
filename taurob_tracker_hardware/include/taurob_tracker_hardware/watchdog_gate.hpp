// Copyright (c) 2025 taurob GmbH. All rights reserved.
// 3-clause BSD; see COPYING.

#ifndef TAUROB_TRACKER_HARDWARE__WATCHDOG_GATE_HPP_
#define TAUROB_TRACKER_HARDWARE__WATCHDOG_GATE_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace taurob_tracker_hardware
{

/// Gates hardware actuation on the taurob safety watchdog.
///
/// Subscribes to the watchdog_feed topic (published by the taurob_watchdog
/// node, which relays pulses from the watchdog client on the control station)
/// and tracks whether a pulse has arrived within the timeout window. Hardware
/// interfaces must not send motion commands to the ECUs unless is_ok() is true.
class WatchdogGate
{
public:
  explicit WatchdogGate(
    const std::string & topic = "watchdog_feed",
    std::chrono::milliseconds timeout = std::chrono::milliseconds(1000))
  : timeout_(timeout)
  {
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
      owned_context_ = true;
    }
    node_ = std::make_shared<rclcpp::Node>("taurob_watchdog_gate");
    subscription_ = node_->create_subscription<std_msgs::msg::Bool>(
      topic, 1,
      [this](const std_msgs::msg::Bool::SharedPtr msg)
      {
        if (msg->data)
        {
          last_pulse_ = std::chrono::steady_clock::now();
          received_once_ = true;
        }
      });
    executor_.add_node(node_);
    spin_thread_ = std::thread([this]() {executor_.spin();});
  }

  ~WatchdogGate()
  {
    executor_.cancel();
    if (spin_thread_.joinable())
    {
      spin_thread_.join();
    }
    if (owned_context_)
    {
      rclcpp::shutdown();
    }
  }

  WatchdogGate(const WatchdogGate &) = delete;
  WatchdogGate & operator=(const WatchdogGate &) = delete;

  /// True if a watchdog pulse has been received within the timeout window.
  bool is_ok() const
  {
    if (!received_once_.load())
    {
      return false;
    }
    const auto since_pulse =
      std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - last_pulse_.load());
    return since_pulse <= timeout_;
  }

  /// True if at least one watchdog pulse was ever received.
  bool received_once() const
  {
    return received_once_.load();
  }

  /// Timeout window (matches the 1 s window of the taurob_watchdog node).
  std::chrono::milliseconds timeout_;
  std::atomic<std::chrono::steady_clock::time_point> last_pulse_;
  std::atomic<bool> received_once_{false};
  bool owned_context_ = false;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spin_thread_;
};

}  // namespace taurob_tracker_hardware

#endif  // TAUROB_TRACKER_HARDWARE__WATCHDOG_GATE_HPP_
