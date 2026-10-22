# Taurob Tracker API — ROS 2 Jazzy Migration Plan

This document is the working plan for porting the driver stack in this repository from ROS 1
(the old `master` branch, and the partially-ported `humble` branch) to **ROS 2 Jazzy**.

Guiding decision (agreed with repo owner): **all driver nodes that do not use `ros2_control` are
deprecated and will be removed / not ported.** The native ROS-topic driver interfaces
(`taurob_base_node`, `taurob_arm_node`, `taurob_flipper_node`, `taurob_claw_node`) are replaced by
`ros2_control` hardware interfaces backed by the same `libtaurob` low-level communication library.

## Current state (humble branch)

| Package | Type | Status on `humble` |
|---|---|---|
| `libtaurob` | library (arm, base, flipper, tools, sockets) | ✅ ported to ament_cmake / rclcpp |
| `taurob_tracker_description` | URDF, meshes, RViz launch | ✅ ported to ament_cmake |
| `taurob_base_node` | driver node (native topics) | ❌ still catkin / ROS 1 | 
| `taurob_arm_node` | driver node (native topics) | ❌ still catkin / ROS 1 |
| `taurob_flipper_node` | driver node (native topics) | ❌ still catkin / ROS 1 |
| `taurob_commander_switch` | ROS/Commander multiplexer | ❌ still catkin / ROS 1 |
| `taurob_relay_dropper` | relay dropper node | ❌ still catkin / ROS 1 |
| `taurob_teleop_twist_joy` | joystick teleop | ❌ still catkin / ROS 1 |
| `taurob_watchdog` | watchdog server node | ❌ still catkin / ROS 1 |
| `taurob_watchdog_client` | standalone watchdog client | ❌ still catkin / ROS 1 |

ROS 1 `master` additionally contains `taurob_arm_ros_control` and `taurob_flipper_ros_control`
(ros_control hardware interfaces) plus `taurob_bringup`, `taurob_robot_description`,
`taurob_arm_description`, and Gazebo packages. The ros_control variants are the reference for the
ROS 2 `ros2_control` implementation, but their code is also ROS 1 and requires full rework.

## Target architecture (jazzy)

- **`libtaurob`** — unchanged public API; modernize only where needed (rclcpp logging, QoS, removed
  ROS 1 APIs). This is the single low-level gateway to the taurob U-ECUs (TCP/UDP sockets).
- **`taurob_tracker_hardware`** (new) — a `ros2_control` hardware interface package containing
  System/HardwareInterface implementations for:
  - **Base platform** — tracked diff-drive. Exposes `wheel_left`/`wheel_right` (or track) joint
    velocity commands and odometry state via `ros2_control` diff_drive_controller; battery voltage,
    temperature, IMU, error codes exported as `ros2_control` sensors / GPIO interfaces where
    appropriate.
  - **Arm** — position-controlled joints (`arm_joint_0..N`), calibrated via the existing
    per-joint offset YAML config (port `taurob_arm_node/config/tracker.yaml`).
  - **Flippers** — position-controlled joints (`flipper_front`, `flipper_rear` if applicable).
  - **Claw** — position joints `claw_rotation`, `claw_grip` + grip-force sensor interface (base on
    the ROS 1 `taurob_claw_ros_control` work merged into `master`).
- **`taurob_tracker_description`** — keep, extend the URDF with a `<ros2_control>` block matching
  the hardware interface joints (needed by `controller_manager`).
- **`taurob_bringup`** (new, ROS 2) — launch files (Python) + controller YAML configs
  (`diff_drive_controller`, `joint_trajectory_controller` for arm and flippers, `gripper` controllers)
  + `robot_state_publisher`.
- **`taurob_watchdog` / `taurob_watchdog_client`** — ported as-is (safety-critical, no ros2_control
  dependency). Watchdog feeds the hardware interface to gate actuation; watchdog server stays a plain
  node; client remains ROS-free. **Port is mandatory before any hardware operation.**
- **Deprecated, removed (not ported):** `taurob_base_node`, `taurob_arm_node`, `taurob_flipper_node`
  (all `*_ros_control` ROS 1 packages), `taurob_commander_switch`, `taurob_relay_dropper`
  (ROS 1 python nodes; revisit only if the smokebot payload returns), `taurob_robot_gazebo` ROS 1
  packages.
- **`taurob_teleop_twist_joy`** — drop in favor of the upstream ROS 2 `teleop_twist_joy` release;
  keep the taurob joystick config YAMLs only if they are still useful.

## Porting order and work breakdown

### Phase 0 — Branch hygiene
- [x] Create `jazzy` branch from `humble`.
- [x] Delete the unported ROS 1 node packages that are now deprecated
      (`taurob_base_node`, `taurob_arm_node`, `taurob_flipper_node`, `taurob_commander_switch`,
      `taurob_relay_dropper`, `taurob_teleop_twist_joy`).
- [x] Update top-level README: ROS 2 Jazzy-only, build with `colcon build`.

### Phase 1 — Foundation
- [ ] Verify/patch `libtaurob` builds on Jazzy (rclcpp API drift from Humble; check deprecation
      warnings, `diagnostic_updater` availability).
- [ ] Verify `taurob_tracker_description` builds and `visualize.launch.py` works on Jazzy.
- [ ] Add CI job (GitHub Actions) building the workspace on `ros:jazzy` base image.

### Phase 2 — Safety stack
- [ ] Port `taurob_watchdog` (C++ node → rclcpp, `std_msgs/Bool` on `watchdog_feed`; parameters as
      ROS 2 parameters; launch file → Python).
- [ ] Port `taurob_watchdog_client` (plain C++/CMake, no ROS dependency; only build-system cleanup).
- [ ] Integrate watchdog gate into the hardware interface `on_activate`/`write()` path: no ECU
      commands unless `watchdog_feed` pulses arrive (< 150 ms gap, as in ROS 1 implementation).

### Phase 3 — ros2_control hardware interfaces
- [ ] Create `taurob_tracker_hardware` package (ament_cmake, depends on `hardware_interface`,
      `rclcpp`, `libtaurob`).
- [ ] Implement `TrackerBaseSystem` : `hardware_interface::SystemInterface`
      - `on_init`/`on_configure`: params `base_ip`, `base_port`, `protocol_version`, track geometry
        (`track_width`, `wheel_diameter`), TF frames.
      - `read()`: odometry, supply voltage, error code, remaining optime, aux bits, IMU (export
        voltage/optime as GPIO/sensor interfaces; IMU optionally via a separate broadcaster node
        if `ros2_control` IMU sensor interface is awkward).
      - `write()`: velocity commands to tracks, headlight/bluelight as GPIO commands.
- [ ] Implement `TrackerArmHardware` : position command/state per arm joint; per-joint offset
      calibration loaded from YAML (port `config/tracker.yaml.default` semantics).
- [ ] Implement `TrackerFlipperHardware` : position command/state, neutral-pose semantics.
- [ ] Implement `TrackerClawHardware` : position joints + force sensor state.
- [ ] Unit-test all interfaces against a mock ECU endpoint (loopback UDP/TCP server) — no hardware
      in CI.

### Phase 4 — Controllers and bringup
- [ ] Add `<ros2_control>` section to `tracker.urdf.xacro` with a hardware-parameterized plugin
      selection (real vs. mock/forward-only).
- [ ] `taurob_bringup` package: 
      - `tracker_bringup.launch.py` (robot_state_publisher, controller_manager, hardware spawn),
      - controller configs: `diff_drive_controller` (base), `joint_trajectory_controller` (arm,
        flippers), gripper action/controller for claw,
      - watchdog server included by default,
      - keep `tracker.yaml.default` pattern for arm calibration to preserve user-modified files.
- [ ] Port smoke tests: `ros2 launch` bringup against mock ECU; `ros2 topic pub /cmd_vel` and
      `ros2 control` command verification.

### Phase 5 — Teleop and peripherals
- [ ] Use upstream `teleop_twist_joy` (ROS 2) for driving; add taurob joy config if needed.
- [ ] Re-evaluate `taurob_relay_dropper`/`taurob_commander_switch` only on explicit request
      (smokebot-specific hardware); default: leave out of the ROS 2 stack.

### Phase 6 — Docs and release
- [ ] Rewrite README (Jazzy): prerequisites (chrony NTP sync, `hector_xacro_tools`), build
      instructions, launch instructions, controller cheatsheet (`ros2 topic pub /cmd_vel ...`,
      `ros2 control load_start_controller ...`), battery-voltage safety warnings, watchdog usage.
- [ ] Remove the humble status tracker checklist; document package overview and deprecation notes.
- [ ] Tag a pre-release once hardware validation on a real Tracker is done (cannot be done in CI).

## Safety requirements carried over from ROS 1 (non-negotiable)
1. The watchdog client must run on the control station; the watchdog server on the robot. No
   actuation without watchdog pulses (< 150 ms timeout as in the original implementation).
2. Only one instance of any driver/hardware interface may run at a time.
3. `supply_voltage` must be monitored during operation (< 23 V critical, < 22 V shut down).
4. Arm calibration (`tracker.yaml`) is mandatory before commanding arm joints.
5. Do not use ROS and the taurob Commander software simultaneously (no multiplexer in ROS 2 stack).

## Verification strategy
- `colcon build` + `colcon test` green on `ros:jazzy` for every PR.
- Hardware interfaces tested with mock ECU (loopback sockets) — no dependency on real hardware.
- `ros2 launch taurob_tracker_description visualize.launch.py` for URDF smoke test.
- Final validation checklist on the real robot (drive, flipper, arm, claw, watchdog e-stop), to be
  executed and signed off by the robot maintainer.
