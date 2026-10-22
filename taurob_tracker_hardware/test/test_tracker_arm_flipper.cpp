// Copyright (c) 2025 taurob GmbH. All rights reserved.
// 3-clause BSD; see COPYING.

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "mock_ecu_common.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstring>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "taurob_tracker_hardware/tracker_arm_hardware.hpp"
#include "taurob_tracker_hardware/tracker_flipper_hardware.hpp"

namespace
{

using taurob_tracker_hardware_test::calc_crc16;
using taurob_tracker_hardware_test::find_free_udp_port;

/// Mock of an arm joint U-ECU: parses command frames (13 bytes), records the
/// commanded state and position, replies with a valid status frame (23 bytes)
/// echoing the received sequence number.
class MockArmEcu
{
public:
  explicit MockArmEcu(uint16_t listen_port)
  {
    socket_ = socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(listen_port);
    if (::bind(socket_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0)
    {
      throw std::runtime_error("bind failed");
    }
    running_ = true;
    thread_ = std::thread([this] { loop(); });
  }

  ~MockArmEcu()
  {
    running_ = false;
    if (thread_.joinable())
    {
      thread_.join();
    }
    close(socket_);
  }

  uint8_t commanded_state() const { return state_; }
  int commanded_position_mrad() const { return position_mrad_; }

private:
  void loop()
  {
    unsigned char buffer[64];
    sockaddr_in from{};
    socklen_t from_len = sizeof(from);
    while (running_)
    {
      timeval tv{0, 100000};
      setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
      from_len = sizeof(from);
      const ssize_t n = recvfrom(
        socket_, buffer, sizeof(buffer), 0,
        reinterpret_cast<sockaddr *>(&from), &from_len);
      if (n != 13)
      {
        continue;
      }
      state_ = buffer[3];
      position_mrad_ = (buffer[4] << 8) + buffer[5];

      // reply with status frame (23 bytes), echoing sequence + 1
      unsigned char status[23];
      status[0] = 2;   // status frame id
      status[1] = 1;   // protocol version
      status[2] = static_cast<uint8_t>(buffer[2] + 1);
      status[3] = 0;   // bitfield: no error
      status[4] = status[5] = status[6] = status[7] = 0;  // timestamp s
      status[8] = status[9] = status[10] = status[11] = 0;  // timestamp us
      // report the commanded position as current position
      status[12] = buffer[4];
      status[13] = buffer[5];
      status[14] = 0; status[15] = 0;  // speed
      status[16] = 0; status[17] = 0;  // effort
      status[18] = 0;   // current
      status[19] = 60;  // temperature (0 C + 40)
      status[20] = 180;  // voltage
      const uint16_t crc = calc_crc16(status, 21);
      status[21] = static_cast<unsigned char>(crc >> 8);
      status[22] = static_cast<unsigned char>(crc & 0xFF);
      sendto(
        socket_, status, sizeof(status), 0,
        reinterpret_cast<sockaddr *>(&from), sizeof(from));
    }
  }

  int socket_;
  std::thread thread_;
  std::atomic<bool> running_{false};
  std::atomic<uint8_t> state_{0};
  std::atomic<int> position_mrad_{0};
};

/// Mock of the flipper U-ECU: parses command frames (8 bytes), records the
/// commanded angle, replies with a valid status frame (18 bytes).
class MockFlipperEcu
{
public:
  explicit MockFlipperEcu(uint16_t listen_port)
  {
    socket_ = socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(listen_port);
    if (::bind(socket_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0)
    {
      throw std::runtime_error("bind failed");
    }
    running_ = true;
    thread_ = std::thread([this] { loop(); });
  }

  ~MockFlipperEcu()
  {
    running_ = false;
    if (thread_.joinable())
    {
      thread_.join();
    }
    close(socket_);
  }

  int commanded_angle_mrad() const { return angle_mrad_; }

private:
  void loop()
  {
    unsigned char buffer[64];
    sockaddr_in from{};
    socklen_t from_len = sizeof(from);
    while (running_)
    {
      timeval tv{0, 100000};
      setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
      from_len = sizeof(from);
      const ssize_t n = recvfrom(
        socket_, buffer, sizeof(buffer), 0,
        reinterpret_cast<sockaddr *>(&from), &from_len);
      if (n != 8)
      {
        continue;
      }
      angle_mrad_ = (buffer[4] << 8) + buffer[5];

      unsigned char status[18];
      status[0] = 2;   // status frame id
      status[1] = 1;   // protocol version
      status[2] = static_cast<uint8_t>(buffer[2] + 1);
      status[3] = 0;   // bitfield
      status[4] = status[5] = status[6] = status[7] = 0;  // timestamp s
      status[8] = 0; status[9] = 0;   // timestamp us (2 bytes)
      // report the commanded angle as current angle
      status[10] = buffer[4];
      status[11] = buffer[5];
      status[12] = 0; status[13] = 0;  // speed
      status[14] = 0; status[15] = 0;  // current
      const uint16_t crc = calc_crc16(status, 16);
      status[16] = static_cast<unsigned char>(crc >> 8);
      status[17] = static_cast<unsigned char>(crc & 0xFF);
      sendto(
        socket_, status, sizeof(status), 0,
        reinterpret_cast<sockaddr *>(&from), sizeof(from));
    }
  }

  int socket_;
  std::thread thread_;
  std::atomic<bool> running_{false};
  std::atomic<int> angle_mrad_{0};
};

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
};

std::string arm_hardware_info_xml(int port, const std::string & watchdog)
{
  return
    "<?xml version=\"1.0\"?>"
    "<robot name=\"tracker_test\">"
    "<link name=\"base_link\"/><link name=\"a0\"/><link name=\"a1\"/>"
    "<joint name=\"arm_joint_0\" type=\"revolute\">"
    "<parent link=\"base_link\"/><child link=\"a0\"/></joint>"
    "<joint name=\"arm_joint_1\" type=\"revolute\">"
    "<parent link=\"a0\"/><child link=\"a1\"/></joint>"
    "  <ros2_control name=\"TestArm\" type=\"system\">"
    "    <hardware>"
    "      <plugin>taurob_tracker_hardware/TrackerArmHardware</plugin>"
    "      <param name=\"watchdog_enabled\">" + watchdog + "</param>"
    "    </hardware>"
    "    <joint name=\"arm_joint_0\">"
    "      <command_interface name=\"position\"/>"
    "      <state_interface name=\"position\"/>"
    "      <param name=\"ip\">127.0.0.1</param>"
    "      <param name=\"port\">" + std::to_string(port) + "</param>"
    "      <param name=\"position_tolerance_mrad\">100</param>"
    "      <param name=\"offset\">0</param>"
    "    </joint>"
    "    <joint name=\"arm_joint_1\">"
    "      <command_interface name=\"position\"/>"
    "      <state_interface name=\"position\"/>"
    "      <param name=\"ip\">127.0.0.1</param>"
    "      <param name=\"port\">" + std::to_string(port) + "</param>"
    "      <param name=\"position_tolerance_mrad\">100</param>"
    "      <param name=\"offset\">0</param>"
    "    </joint>"
    "  </ros2_control>"
    "</robot>";
}

std::string flipper_hardware_info_xml(int port, const std::string & watchdog)
{
  return
    "<?xml version=\"1.0\"?>"
    "<robot name=\"tracker_test\">"
    "<link name=\"base_link\"/><link name=\"f0\"/>"
    "<joint name=\"flipper_front\" type=\"revolute\">"
    "<parent link=\"base_link\"/><child link=\"f0\"/></joint>"
    "  <ros2_control name=\"TestFlipper\" type=\"system\">"
    "    <hardware>"
    "      <plugin>taurob_tracker_hardware/TrackerFlipperHardware</plugin>"
    "      <param name=\"flipper_ip\">127.0.0.1</param>"
    "      <param name=\"flipper_port\">" + std::to_string(port) + "</param>"
    "      <param name=\"watchdog_enabled\">" + watchdog + "</param>"
    "    </hardware>"
    "    <joint name=\"flipper_front\">"
    "      <command_interface name=\"position\"/>"
    "      <state_interface name=\"position\"/>"
    "    </joint>"
    "  </ros2_control>"
    "</robot>";
}

/// The arm interface forwards position commands to the (mock) ECU.
TEST_F(TestFixture, ArmHardwareCommandsMockEcu)
{
  const uint16_t port = find_free_udp_port();
  MockArmEcu ecu(port);

  const auto info = hardware_interface::parse_control_resources_from_urdf(
    arm_hardware_info_xml(port, "false"));

  auto arm = std::make_shared<taurob_tracker_hardware::TrackerArmHardware>();
  ASSERT_EQ(arm->on_init(info[0]), hardware_interface::CallbackReturn::SUCCESS);

  auto commands = arm->export_command_interfaces();
  ASSERT_EQ(commands.size(), 2u);

  const rclcpp_lifecycle::State state;
  ASSERT_EQ(arm->on_activate(state), hardware_interface::CallbackReturn::SUCCESS);

  commands[0].set_value(0.5);  // rad
  commands[1].set_value(-0.25);

  const rclcpp::Time now(0, 0, RCL_ROS_TIME);
  const rclcpp::Duration dt = rclcpp::Duration::from_seconds(0.05);
  for (int i = 0; i < 30 && (ecu.commanded_position_mrad() == 0); ++i)
  {
    ASSERT_EQ(arm->read(now, dt), hardware_interface::return_type::OK);
    ASSERT_EQ(arm->write(now, dt), hardware_interface::return_type::OK);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  // 0.5 rad = 500 mrad on the wire
  EXPECT_EQ(ecu.commanded_position_mrad(), 500);
  EXPECT_EQ(ecu.commanded_state(), 1);  // POSITION_CONTROL

  auto states = arm->export_state_interfaces();
  ASSERT_EQ(states.size(), 2u);
  // read back the echoed position
  for (int i = 0; i < 20 && std::abs(states[0].get_value()) < 0.4; ++i)
  {
    arm->read(now, dt);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  EXPECT_NEAR(states[0].get_value(), 0.5, 0.01);

  ASSERT_EQ(arm->on_deactivate(state), hardware_interface::CallbackReturn::SUCCESS);
}

/// With the watchdog enabled but not fed, the arm must command STOP.
TEST_F(TestFixture, ArmHardwareStopsWhenWatchdogNotFed)
{
  const uint16_t port = find_free_udp_port();
  MockArmEcu ecu(port);

  const auto info = hardware_interface::parse_control_resources_from_urdf(
    arm_hardware_info_xml(port, "true"));

  auto arm = std::make_shared<taurob_tracker_hardware::TrackerArmHardware>();
  ASSERT_EQ(arm->on_init(info[0]), hardware_interface::CallbackReturn::SUCCESS);

  auto commands = arm->export_command_interfaces();
  const rclcpp_lifecycle::State state;
  ASSERT_EQ(arm->on_activate(state), hardware_interface::CallbackReturn::SUCCESS);

  commands[0].set_value(0.5);
  commands[1].set_value(0.5);

  const rclcpp::Time now(0, 0, RCL_ROS_TIME);
  const rclcpp::Duration dt = rclcpp::Duration::from_seconds(0.05);
  for (int i = 0; i < 10; ++i)
  {
    ASSERT_EQ(arm->read(now, dt), hardware_interface::return_type::OK);
    ASSERT_EQ(arm->write(now, dt), hardware_interface::return_type::OK);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  EXPECT_EQ(ecu.commanded_state(), 0);  // STOP

  ASSERT_EQ(arm->on_deactivate(state), hardware_interface::CallbackReturn::SUCCESS);
}

/// The flipper interface forwards position commands to the (mock) ECU.
TEST_F(TestFixture, FlipperHardwareCommandsMockEcu)
{
  const uint16_t port = find_free_udp_port();
  MockFlipperEcu ecu(port);

  const auto info = hardware_interface::parse_control_resources_from_urdf(
    flipper_hardware_info_xml(port, "false"));

  auto flipper = std::make_shared<taurob_tracker_hardware::TrackerFlipperHardware>();
  ASSERT_EQ(flipper->on_init(info[0]), hardware_interface::CallbackReturn::SUCCESS);

  auto commands = flipper->export_command_interfaces();
  ASSERT_EQ(commands.size(), 1u);

  const rclcpp_lifecycle::State state;
  ASSERT_EQ(flipper->on_activate(state), hardware_interface::CallbackReturn::SUCCESS);

  commands[0].set_value(0.3);  // rad

  const rclcpp::Time now(0, 0, RCL_ROS_TIME);
  const rclcpp::Duration dt = rclcpp::Duration::from_seconds(0.05);
  for (int i = 0; i < 30 && (ecu.commanded_angle_mrad() == 0); ++i)
  {
    ASSERT_EQ(flipper->read(now, dt), hardware_interface::return_type::OK);
    ASSERT_EQ(flipper->write(now, dt), hardware_interface::return_type::OK);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  // 0.3 rad = 300 mrad on the wire
  EXPECT_EQ(ecu.commanded_angle_mrad(), 300);

  ASSERT_EQ(flipper->on_deactivate(state), hardware_interface::CallbackReturn::SUCCESS);
}

/// With the watchdog enabled but not fed, the flipper must hold its position.
TEST_F(TestFixture, FlipperHardwareHoldsWhenWatchdogNotFed)
{
  const uint16_t port = find_free_udp_port();
  MockFlipperEcu ecu(port);

  const auto info = hardware_interface::parse_control_resources_from_urdf(
    flipper_hardware_info_xml(port, "true"));

  auto flipper = std::make_shared<taurob_tracker_hardware::TrackerFlipperHardware>();
  ASSERT_EQ(flipper->on_init(info[0]), hardware_interface::CallbackReturn::SUCCESS);

  auto commands = flipper->export_command_interfaces();
  const rclcpp_lifecycle::State state;
  ASSERT_EQ(flipper->on_activate(state), hardware_interface::CallbackReturn::SUCCESS);

  commands[0].set_value(0.3);

  const rclcpp::Time now(0, 0, RCL_ROS_TIME);
  const rclcpp::Duration dt = rclcpp::Duration::from_seconds(0.05);
  for (int i = 0; i < 10; ++i)
  {
    ASSERT_EQ(flipper->read(now, dt), hardware_interface::return_type::OK);
    ASSERT_EQ(flipper->write(now, dt), hardware_interface::return_type::OK);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  // the mock reports the commanded angle; with the watchdog unfed the
  // interface must have held the initial position (0) instead
  EXPECT_EQ(ecu.commanded_angle_mrad(), 0);

  ASSERT_EQ(flipper->on_deactivate(state), hardware_interface::CallbackReturn::SUCCESS);
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
