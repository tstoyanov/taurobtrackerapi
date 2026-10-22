# taurobtrackerapi

Drivers for the taurob tracker robot system, ported to **ROS 2 Jazzy**.

## Status

The ROS 2 stack is complete at the software level. All hardware access goes through
[ros2_control](https://control.ros.org/) hardware interfaces backed by the `libtaurob`
low-level communication library. **Validation on real robot hardware is still pending**
(see the checklist in [MIGRATION_PLAN.md](MIGRATION_PLAN.md)).

| Package | Description | Status |
|---|---|---|
| `libtaurob` | low-level U-ECU communication (base, arm, flipper, claw, sockets) | ✅ ported |
| `taurob_tracker_description` | URDF/xacro, meshes, RViz launch, ros2_control tags | ✅ ported |
| `taurob_tracker_hardware` | ros2_control hardware interfaces (base, arm, flipper, claw) | ✅ ported, robot validation pending |
| `taurob_watchdog` | watchdog server node (safety) | ✅ ported |
| `taurob_watchdog_client` | standalone watchdog client (control station, no ROS deps) | ✅ ported |
| `taurob_bringup` | launch file + controller configurations | ✅ new |

The old ROS 1 native-interface driver nodes are deprecated and removed.

## Architecture

* **Base platform** — diff-drive tracks via `diff_drive_controller`: `/cmd_vel` in, odometry out.
* **Arm** — position-controlled joints via `joint_trajectory_controller`, per-joint ECU
  addressing and offset calibration in the URDF parameters.
* **Flipper / claw** — position-controlled joints via `joint_trajectory_controller`;
  the claw also exposes a grip-force sensor state.
* **Safety** — the watchdog server (robot) + watchdog client (control station) gate all
  actuation: every hardware interface refuses to send motion commands to the ECUs unless
  watchdog pulses arrive within the timeout window.

## Prerequisites

* ROS 2 Jazzy with `ros2_control` (`ros-jazzy-ros2-control`, `ros-jazzy-ros2-controllers`).
* NTP-synchronized clocks between the robot ECUs and the ROS computer (chrony example in
  the migration plan).
* [hector_xacro_tools](https://github.com/tu-darmstadt-ros-pkg/hector_models) for the URDF
  (ROS 1 package of pure xacro macros — stage or overlay it).

## Compilation

```bash
colcon build
source install/setup.bash
```

## Usage

Visualize the robot model:

```bash
ros2 launch taurob_tracker_description visualize.launch.py
```

Full robot bringup (on the robot PC; watchdog client must run on the control station):

```bash
ros2 launch taurob_bringup tracker_bringup.launch.py
```

Driving the robot (watchdog client running, otherwise motion commands are gated):

```bash
ros2 topic pub /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist '{linear: {x: 0.3}, angular: {z: -0.1}}' -r 10
```

Watchdog client on the control station (press any key for emergency stop):

```bash
ros2 run taurob_watchdog_client taurob_watchdog_client <robot_ip>
```

## Safety warnings

*WARNING:* Using the watchdog is *mandatory* for operating the robot with ROS. The watchdog
client must run on the control station, not on the robot.

*WARNING:* Do not run multiple instances of any driver or hardware interface! The resulting
actions are unforeseeable. Do not use ROS and the taurob Commander software together.

*WARNING:* Monitor the published supply voltage during operation. Stop using the robot if it
drops below 23 V permanently; below 22 V shut everything down and charge the battery to avoid
deep-discharge damage.

*WARNING:* If your robot has an arm, it MUST be calibrated (per-joint offset parameters in the
URDF) before using it with ROS.

## License

Copyright (c) 2016-2025 taurob GmbH. BSD 3-clause; see [COPYING](COPYING).
