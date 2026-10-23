# Smokebot Baseline — Startup and Testing Guide (ROS 1)

This guide describes how to bring up and test the ROS 1 baseline stack on the `master`
branch (the upstream `smokebot` code line). It is the reference setup for validating the
robot before testing the ROS 2 migration on the `jazzy` branch.

## Prerequisites

1. **ROS 1** (e.g. Noetic) with a catkin workspace containing these packages:
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/tstoyanov/taurobtrackerapi.git
   cd ~/catkin_ws
   rosdep install --from-paths src --ignore-src -y
   catkin build
   source devel/setup.bash
   ```
2. **hector_xacro_tools** (URDF dependency):
   ```bash
   git clone https://github.com/tu-darmstadt-ros-pkg/hector_models.git ~/catkin_ws/src/hector_models
   ```
3. **NTP-synchronized clocks** between the robot ECUs and the ROS computer (chrony
   example configuration in the main README).
4. **Network**: the robot's internal network reachable (default ECU addresses below).
5. Never run two instances of any node, and do not run ROS and the taurob Commander
   software at the same time (use the commander switch instead).

## Robot configuration (defaults in the launch files)

| ECU | IP | Port | Configured in |
|---|---|---|---|
| Base drivetrain | `10.0.0.2` | `8080` | `taurob_base_node/launch/smokebot.launch` |
| Flipper | `10.0.0.41` | `1111` | `taurob_flipper_node/launch/smokebot.launch` |
| Arm joint 0 | `10.0.0.106` | `8041` | `taurob_arm_node/config/tracker.yaml` |
| Arm joint 1 | `10.0.0.107` | `8042` | `taurob_arm_node/config/tracker.yaml` |
| Arm joint 2 | `10.0.0.107` | `8043` | `taurob_arm_node/config/tracker.yaml` |
| Arm joint 3 | `10.0.0.108` | `8044` | `taurob_arm_node/config/tracker.yaml` |

Verify these match your robot's ECU map before starting; edit the launch files /
`tracker.yaml` if they differ. The arm config format is
`name:ip_addr:port:offset(rad)`.

## Startup sequence

**1. Watchdog server (on the robot PC):**

```bash
roslaunch taurob_watchdog watchdog.launch
```

**2. Driver nodes (on the robot PC):**

```bash
# base platform (drivetrain)
roslaunch taurob_base_node smokebot.launch

# flipper
roslaunch taurob_flipper_node smokebot.launch

# arm
roslaunch taurob_arm_node tracker.launch
```

The base and flipper launches remap their command topics to `cmd_vel_switched` and
`jointstate_cmd_switched` respectively, so commands only take effect while the
commander switch routes control to ROS.

**3. Watchdog client (MANDATORY — on the control station, not the robot):**

```bash
rosrun taurob_watchdog_client taurob_watchdog_client <robot_ip>
```

No motion is possible until the watchdog client feeds pulses (press any key in the
client terminal to trigger an emergency stop).

**4. (Optional) Commander switch** — to hand control between ROS and the taurob
Commander software:

```bash
roslaunch taurob_commander_switch smokebot.launch        # switch node
roslaunch taurob_commander_switch smokebot-gui.launch    # selection GUI
```

**5. (Optional) Relay dropper** (smokebot payload):

```bash
roslaunch taurob_relay_dropper smokebot.launch
```

**6. (Optional) Teleop:**

```bash
roslaunch taurob_teleop_twist_joy teleop.launch
```

## Testing the baseline

Perform these checks in order, with the watchdog client running:

### 1. Communication and status
```bash
rostopic hz /odom                 # base odometry stream (should be steady)
rostopic echo /supply_voltage     # battery voltage, expect ~23-28 V
rostopic echo /error_code         # 0 = no error
rostopic echo /joint_states       # arm joint and flipper positions
```

### 2. Watchdog / emergency stop
Stop the watchdog client (Ctrl+C or any keypress). The robot must not accept any
motion command until the client is restarted. Verify by sending a drive command
(next step) — it must be ignored.

### 3. Driving the base
**CAREFUL:** the robot will not stop until a zero command is sent or the watchdog
client is stopped.
```bash
# slow forward, 0.1 m/s
rostopic pub /cmd_vel_switched geometry_msgs/Twist -r 10 -- '[0.1, 0.0, 0.0]' '[0.0, 0.0, 0.0]'
# stop
rostopic pub /cmd_vel_switched geometry_msgs/Twist -r 10 -- '[0.0, 0.0, 0.0]' '[0.0, 0.0, 0.0]'
```
Verify odometry moves consistently with the actual motion; verify the robot stops
cleanly on zero command.

### 4. Flipper
```bash
rostopic pub /jointstate_cmd_switched sensor_msgs/JointState \
  '{name: ["flipper_front"], position: [0.3]}' -1
```
Verify the flipper moves to the commanded angle and reports it on `/joint_states`
(0 = the pose the flipper had at power-on).

### 5. Arm
```bash
# command all joints to the upright pose (adjust per your arm's calibration)
rostopic pub /jointstate_cmd sensor_msgs/JointState \
  '{name: ["arm_joint_0", "arm_joint_1", "arm_joint_2", "arm_joint_3"],
    position: [0.0, 0.0, 0.0, 0.0]}' -1
```
Verify the reported positions on `/joint_states` converge to the commanded ones and
match the physical arm. Verify per-joint offsets in `taurob_arm_node/config/tracker.yaml`
against a known reference pose if they do not.

### 6. Visualization (sanity check of the model)
```bash
roslaunch taurob_tracker_description tracker_model_rviz.launch
```

## Troubleshooting

* **No status on /odom or /joint_states**: check ECU IPs/ports (table above), and the
  NTP sync (timestamps with unsynchronized clocks are rejected).
* **Commands ignored**: is the watchdog client running on the control station? Is the
  commander switch routing control to ROS?
* **Arm unresponsive after a friction-clutch event**: power-cycle the affected arm
  ECU (known upstream behavior).
* **Voltage below 23 V**: stop testing and charge the battery; below 22 V shut
  everything down (deep-discharge risk).

When all checks pass, the baseline is validated and the same procedure can be used to
test the ROS 2 migration — see `ROBOT_TESTING_JAZZY.md` on the `jazzy` branch.
