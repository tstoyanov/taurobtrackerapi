# taurobtrackerapi

This repository contains drivers for the taurob tracker robot system, being ported to **ROS 2 Jazzy**.

## Status

The old ROS 1 native-interface driver nodes (`taurob_base_node`, `taurob_arm_node`,
`taurob_flipper_node`, `taurob_commander_switch`, `taurob_relay_dropper`, `taurob_teleop_twist_joy`)
are **deprecated and removed**. The ROS 2 stack is being rebuilt around
[ros2_control](https://control.ros.org/) hardware interfaces backed by the `libtaurob` low-level
communication library.

See [MIGRATION_PLAN.md](MIGRATION_PLAN.md) for the detailed migration plan and progress.

| Package | Type | Status |
|---|---|---|
| `libtaurob` | low-level U-ECU communication library (arm, base, flipper, sockets) | ✅ ported to ament_cmake |
| `taurob_tracker_description` | URDF/xacro, meshes, RViz launch | ✅ ported to ament_cmake |
| `taurob_watchdog` | watchdog server node (safety) | ✅ ported to rclcpp |
| `taurob_watchdog_client` | standalone watchdog client (control station, no ROS deps) | ✅ ported to ament_cmake |
| `taurob_tracker_hardware` | ros2_control hardware interfaces (base, arm, flipper, claw) | 🚧 planned |
| `taurob_bringup` | ROS 2 launch files + controller configs | 🚧 planned |

## Target architecture

All hardware access goes through `ros2_control`:

* **Base platform** — diff-drive velocity commands and odometry via `diff_drive_controller`;
  battery voltage, temperature, error codes and IMU exposed as sensor/GPIO interfaces.
* **Arm / flippers / claw** — position-controlled joints via `joint_trajectory_controller` /
  gripper controllers, with per-joint offset calibration (YAML), same semantics as the old
  `taurob_arm_node` calibration files.
* **Safety** — the watchdog server (robot) + watchdog client (control station) gate all actuation:
  if no watchdog pulse arrives within 150 ms, the hardware interfaces stop communicating with the
  robot, bringing it to a safe halted state.

## Prerequisites

* ROS 2 Jazzy with `ros2_control` (`ros-jazzy-ros2-control`, `ros-jazzy-ros2-controllers`).
* The clocks of the control units inside the robot need to be synchronized via NTP with the
  computer running the ROS stack. Example chrony server configuration:

```
server ntp.ubuntu.com iburst

bindaddress 10.0.0.100
allow 10.0.0.0/24

local stratum 10
```

Start with: `chronyd -n -f chrony.conf`.

* The URDF model in this repository depends on
  [hector_xacro_tools](https://github.com/tu-darmstadt-ros-pkg/hector_models).

## Compilation

Place the packages in your ROS 2 workspace and build with `colcon`:

```bash
colcon build
source install/setup.bash
```

## Usage

Visualize the robot model (works today):

```bash
ros2 launch taurob_tracker_description visualize.launch.py
```

Full robot bringup (available once the `taurob_tracker_hardware` and `taurob_bringup` packages
land, see the migration plan):

```bash
ros2 launch taurob_bringup tracker_bringup.launch.py
```

Driving the robot (with the watchdog client running on the control station):

```bash
ros2 topic pub /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist '{linear: {x: 0.3}, angular: {z: -0.1}}' -r 10
```

## Safety warnings

*WARNING:* Using the watchdog as described above is *mandatory* for operating the robot with ROS.
The watchdog client must run on the control station, not on the robot.

*WARNING:* Do not run multiple instances of any driver or hardware interface! The resulting
actions are unforeseeable. Do not use ROS and the taurob Commander software together.

*WARNING:* Monitor the published supply voltage during operation. Stop using the robot if it drops
below 23 V permanently; below 22 V shut everything down and charge the battery to avoid
deep-discharge damage.

*WARNING:* If your robot has an arm, it MUST be calibrated (per-joint offset YAML) before using it
with ROS.

## License

Copyright (c) 2016-2025 taurob GmbH. BSD 3-clause; see [COPYING](COPYING).
