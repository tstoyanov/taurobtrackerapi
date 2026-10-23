# ROS 2 (jazzy branch) — Real-Robot Testing Guide

This guide describes how to validate the ROS 2 Jazzy migration (`jazzy` branch) on the
real robot. It mirrors the ROS 1 baseline procedure (`ROBOT_TESTING_BASELINE.md` on the
`master` branch) — validate the baseline first, then this stack.

**Before you start:** the software-level port is complete and CI-verified (all hardware
plugins load, watchdog gating unit-tested against a mock ECU), but it has never talked to
real ECUs. Treat the first bring-up as an experiment: wheels off the ground if possible,
emergency stop within reach.

## Prerequisites

1. **ROS 2 Jazzy** with ros2_control:
   ```bash
   sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
     ros-jazzy-diff-drive-controller ros-jazzy-joint-trajectory-controller \
     ros-jazzy-joint-state-broadcaster
   ```
2. **Workspace**:
   ```bash
   mkdir -p ~/ws_jazzy/src && cd ~/ws_jazzy/src
   git clone -b jazzy https://github.com/tstoyanov/taurobtrackerapi.git
   cd ~/ws_jazzy
   rosdep install --from-paths src --ignore-src -y
   colcon build
   source install/setup.bash
   ```
3. **hector_xacro_tools** (ROS 1 package of pure xacro macros — build it in the same
   workspace from the [hector_models](https://github.com/tu-darmstadt-ros-pkg/hector_models)
   repository, or overlay/stage its share directory on `AMENT_PREFIX_PATH`).
4. **NTP-synchronized clocks** (chrony, see the baseline guide) and robot network
   reachability, as in ROS 1.

## Robot configuration

ECU addresses live in `taurob_tracker_description/urdf/tracker_ros2_control.urdf.xacro`
(as ros2_control `<param>`s). Before the first start, verify they match your robot:

| ECU | Default in URDF | Set via |
|---|---|---|
| Base drivetrain | `base_ip` = `10.0.0.2`, `base_port` = `8080` | hardware params |
| Flipper | `flipper_ip` = `10.0.0.2`, `flipper_port` = `10002` | hardware params |
| Claw | `claw_ip` = `10.0.0.2`, `claw_port` = `10003` | hardware params |
| Arm joints 0-3 | `10.0.0.121:9081-9084` | per-joint `ip`/`port` params |

**Note:** the arm defaults follow the old ROS 1 `master` addressing. The smokebot
baseline uses `10.0.0.106-108:8041-8044` (see `taurob_arm_node/config/tracker.yaml` on
`master`) — if your robot is the smokebot variant, update the four `<param name="ip">`
/ `<param name="port">` entries in the URDF to match. Per-joint calibration offsets are
the `<param name="offset">` entries (rad).

## Startup sequence

**1. Full bringup (on the robot PC):**

```bash
ros2 launch taurob_bringup tracker_bringup.launch.py
```

This starts the controller manager, `robot_state_publisher`, the watchdog server, and
spawns the controllers:
- `joint_state_broadcaster`
- `diff_drive_controller` (base tracks)
- `arm_controller`, `flipper_controller`, `claw_controller` (position trajectory control)

Check health:

```bash
ros2 control list_controllers          # all should be 'active'
ros2 control list_hardware_components   # all should be 'active' or 'unconfigured'
```

If hardware stays `unconfigured`/`inactive`, check the ECU addresses and NTP sync —
the interfaces log the reason.

**2. Watchdog client (MANDATORY — on the control station, ROS-free binary):**

```bash
ros2 run taurob_watchdog_client taurob_watchdog_client <robot_ip>
```

Press any key in the client for an emergency stop. **No motion is possible without
watchdog pulses**: all four hardware interfaces gate their write paths on the
`watchdog_feed` topic.

## Testing procedure

Work through the real-robot validation checklist in `MIGRATION_PLAN.md` (section
"Real-robot validation checklist"). In summary, in this order:

### 1. Watchdog gating (before any motion)
With the client running:
```bash
ros2 topic echo /watchdog_feed      # pulses at ~10 Hz while the client runs
```
Stop the client → verify `/watchdog_feed` stops within ~1 s and that all actuation is
blocked (send a drive command, next step — it must be ignored). Restart the client to
re-enable.

### 2. Base driving
```bash
# slow forward, 0.1 m/s
ros2 topic pub /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist \
  '{linear: {x: 0.1}, angular: {z: 0.0}}' -r 10
# stop
ros2 topic pub /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist \
  '{linear: {x: 0.0}, angular: {z: 0.0}}' -r 10
```
Verify: odometry (`/diff_drive_controller/odom`) matches actual motion; the robot stops
cleanly on zero command; emergency-stop during motion halts the robot.
Compare odometry plausibility against the ROS 1 baseline.

### 3. Flipper
```bash
ros2 topic pub /flipper_controller/commands trajectory_msgs/msg/JointTrajectory \
  '{joint_names: ["flipper_front"],
    points: [{positions: [0.3], time_from_start: {sec: 2}}]}' -1
```
Verify the flipper reaches the commanded angle (0 = pose at power-on) and
`/joint_states` reports it.

### 4. Arm
```bash
ros2 topic pub /arm_controller/commands trajectory_msgs/msg/JointTrajectory \
  '{joint_names: ["arm_joint_0", "arm_joint_1", "arm_joint_2", "arm_joint_3"],
    points: [{positions: [0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 5}}]}' -1
```
Verify positions converge and match the physical arm. **If angles are off, the per-joint
`offset` calibration params in the URDF need adjusting** — same calibration procedure as
ROS 1 (`tracker.yaml`), now in the URDF. Compare behavior against the baseline arm test.

### 5. Claw
```bash
ros2 topic pub /claw_controller/commands trajectory_msgs/msg/JointTrajectory \
  '{joint_names: ["claw_rotation", "claw_grip"],
    points: [{positions: [0.5, 0.2], time_from_start: {sec: 3}}]}' -1
ros2 topic echo /joint_states    # includes claw force via the force sensor state
```

### 6. Battery and long-run
```bash
ros2 topic echo /supply_voltage   # expect 23-28 V
```
Run 30 min of mixed operation: watch for overcurrent/cooldown states, voltage drops
below 23 V, and lost ECU connections. Below 22 V: shut down and charge.

## Troubleshooting

* **Hardware won't activate**: check ECU addresses in
  `tracker_ros2_control.urdf.xacro` and NTP sync; check `ros2 control list_hardware_components`.
* **Commands ignored while watchdog runs**: verify the client feeds
  `/watchdog_feed` (echo the topic); check that only ONE instance of each node runs.
* **Arm angles wrong**: per-joint `offset` calibration in the URDF (see above).
* **Odometry drift/incorrect scaling**: check `gear_ratio`, `wheel_diameter`,
  `track_width` params in the URDF against the ROS 1 launch values.

## Known limitations of the migration

* No `taurob_commander_switch` equivalent in ROS 2 — ROS and Commander must not run
  together (also true in ROS 1 without the switch).
* Relay dropper / smokebot payload nodes are not ported (per migration plan).
* Arm/flipper/claw behavior could not be integration-tested against mock ECUs (the
  U-ECU protocol assumes driver and ECU on different hosts); the real-robot checklist
  in `MIGRATION_PLAN.md` is the substitute.
