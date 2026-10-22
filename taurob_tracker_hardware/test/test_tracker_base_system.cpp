// Copyright (c) 2025 taurob GmbH. All rights reserved.
// 3-clause BSD; see COPYING.

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "taurob_tracker_hardware/tracker_base_system.hpp"

namespace
{

/// Minimal mock of the tracker base ECU: receives command frames on UDP,
/// records the latest commanded track velocities and replies with status frames.
class MockBaseEcu
{
public:
  explicit MockBaseEcu(uint16_t listen_port)
  {
    socket_ = socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(listen_port);
    if (::bind(socket_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) { throw std::runtime_error("bind failed"); }
    running_ = true;
    thread_ = std::thread([this] { loop(); });
  }

  ~MockBaseEcu()
  {
    running_ = false;
    if (thread_.joinable())
    {
      thread_.join();
    }
    close(socket_);
  }

  // last commanded values (RPM-scaled wire values)
  double commanded_x() const { return x_; }
  double commanded_y() const { return y_; }

private:
  void loop()
  {
    char buffer[512];
    sockaddr_in from{};
    socklen_t from_len = sizeof(from);
    while (running_)
    {
      timeval tv{0, 100000};  // 100 ms
      setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
      from_len = sizeof(from);
      const ssize_t n = recvfrom(
        socket_, buffer, sizeof(buffer) - 1, 0,
        reinterpret_cast<sockaddr *>(&from), &from_len);
      if (n <= 0)
      {
        continue;
      }
      buffer[n] = '\0';
      // command frame: "1 SQ:%u XV:%d YV:%d ..."
      unsigned sq = 0;
      int xv = 0, yv = 0;
      if (sscanf(buffer, "1 SQ:%u XV:%d YV:%d", &sq, &xv, &yv) == 3)
      {
        x_ = xv;
        y_ = yv;
      }
      // reply with a valid status frame so the driver sees communication
      const std::string status =
        "2 SQ:1 S1:0 S2:0 C1:0 C2:0 D1:0 D2:0 GP:0 M1:0 BA:24000 T1:20 T2:20 T3:20 T4:20 PI:0 RO:0 "
        "EC:0 AX:0 AC:0\n";
      sendto(
        socket_, status.c_str(), status.size(), 0,
        reinterpret_cast<sockaddr *>(&from), sizeof(from));
    }
  }

  int socket_;
  std::thread thread_;
  std::atomic<bool> running_{false};
  std::atomic<double> x_{0.0};
  std::atomic<double> y_{0.0};
};

std::string base_hardware_info_xml(const std::string & ip, int port)
{
  return
    "<?xml version=\"1.0\"?>"
    "<robot name=\"tracker_test\">"
    "  <ros2_control name=\"TestBase\" type=\"system\">"
    "    <hardware>"
    "      <plugin>taurob_tracker_hardware/TrackerBaseSystem</plugin>"
    "      <param name=\"base_ip\">" + ip + "</param>"
    "      <param name=\"base_port\">" + std::to_string(port) + "</param>"
    "      <param name=\"gear_ratio\">76.0</param>"
    "      <param name=\"wheel_diameter\">0.303</param>"
    "      <param name=\"track_width\">0.55</param>"
    "      <param name=\"watchdog_enabled\">false</param>"
    "    </hardware>"
    "    <joint name=\"wheel_left\">"
    "      <command_interface name=\"velocity\"/>"
    "      <state_interface name=\"position\"/>"
    "      <state_interface name=\"velocity\"/>"
    "    </joint>"
    "    <joint name=\"wheel_right\">"
    "      <command_interface name=\"velocity\"/>"
    "      <state_interface name=\"position\"/>"
    "      <state_interface name=\"velocity\"/>"
    "    </joint>"
    "  </ros2_control>"
    "</robot>";
}

class TestFixture : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }
  }

  void SetUp() override
  {
    // find a free UDP port for the mock ECU
    int probe = socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = 0;
    ::bind(probe, reinterpret_cast<sockaddr *>(&addr), sizeof(addr));
    sockaddr_in bound{};
    socklen_t len = sizeof(bound);
    getsockname(probe, reinterpret_cast<sockaddr *>(&bound), &len);
    port_ = ntohs(bound.sin_port);
    close(probe);

    ecu_ = std::make_unique<MockBaseEcu>(port_);
  }

  void TearDown() override
  {
    ecu_.reset();
  }

  uint16_t port_ = 0;
  std::unique_ptr<MockBaseEcu> ecu_;
};

/// The base system must initialize, activate and forward wheel velocity
/// commands to the (mock) ECU as track velocities.
TEST_F(TestFixture, BaseSystemCommandsMockEcu)
{
  const auto info = hardware_interface::parse_control_resources_from_urdf(
    base_hardware_info_xml("127.0.0.1", port_));

  auto base = std::make_shared<taurob_tracker_hardware::TrackerBaseSystem>();
  ASSERT_EQ(
    base->on_init(info[0]), hardware_interface::CallbackReturn::SUCCESS);

  auto commands = base->export_command_interfaces();
  ASSERT_EQ(commands.size(), 2u);

  const rclcpp_lifecycle::State state;
  ASSERT_EQ(base->on_activate(state), hardware_interface::CallbackReturn::SUCCESS);

  // forward motion: same positive velocity on both wheels
  commands[0].set_value(3.0);
  commands[1].set_value(3.0);

  const rclcpp::Time now(0, 0, RCL_ROS_TIME);
  const rclcpp::Duration dt = rclcpp::Duration::from_seconds(0.05);
  for (int i = 0; i < 20 && (ecu_->commanded_y() <= 0.0); ++i)
  {
    ASSERT_EQ(base->write(now, dt), hardware_interface::return_type::OK);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  // gear_ratio 76, wheel_diameter 0.303: 0.5 m/s linear => y ~ 2395 RPM
  EXPECT_GT(ecu_->commanded_y(), 1000.0);
  EXPECT_NEAR(ecu_->commanded_x(), 0.0, 1.0);

  // turning: opposite velocities => non-zero x, zero y
  commands[0].set_value(1.0);
  commands[1].set_value(-1.0);
  for (int i = 0; i < 20 && (std::abs(ecu_->commanded_x()) <= 0.0); ++i)
  {
    ASSERT_EQ(base->write(now, dt), hardware_interface::return_type::OK);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  EXPECT_NEAR(ecu_->commanded_y(), 0.0, 1.0);
  EXPECT_NE(ecu_->commanded_x(), 0.0);

  ASSERT_EQ(base->on_deactivate(state), hardware_interface::CallbackReturn::SUCCESS);
}

/// With the watchdog enabled but not fed, no motion command may reach the ECU.
TEST_F(TestFixture, BaseSystemBlocksCommandsWhenWatchdogNotFed)
{
  auto info = hardware_interface::parse_control_resources_from_urdf(
    base_hardware_info_xml("127.0.0.1", port_));

  // enable the watchdog (no pulses will arrive)
  for (auto & param : info[0].hardware_parameters)
  {
    if (param.first == "watchdog_enabled")
    {
      param.second = "true";
    }
  }

  auto base = std::make_shared<taurob_tracker_hardware::TrackerBaseSystem>();
  ASSERT_EQ(base->on_init(info[0]), hardware_interface::CallbackReturn::SUCCESS);

  auto commands = base->export_command_interfaces();
  const rclcpp_lifecycle::State state;
  ASSERT_EQ(base->on_activate(state), hardware_interface::CallbackReturn::SUCCESS);

  commands[0].set_value(3.0);
  commands[1].set_value(3.0);

  const rclcpp::Time now(0, 0, RCL_ROS_TIME);
  const rclcpp::Duration dt = rclcpp::Duration::from_seconds(0.05);
  for (int i = 0; i < 10; ++i)
  {
    ASSERT_EQ(base->write(now, dt), hardware_interface::return_type::OK);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  // the ECU must have received zero motion commands (or nothing at all)
  EXPECT_NEAR(ecu_->commanded_y(), 0.0, 1.0);
  EXPECT_NEAR(ecu_->commanded_x(), 0.0, 1.0);

  ASSERT_EQ(base->on_deactivate(state), hardware_interface::CallbackReturn::SUCCESS);
}

}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleMock(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
