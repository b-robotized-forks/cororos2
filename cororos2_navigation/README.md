# cororos2_navigation

This package provides cororos-specific launch files and configs for Nav2 with `slam_toolbox`.

## What it adds

- `cororos2_nav2_slam.launch.xml`
- `cororos2_teleop_mux.launch.xml`
- robot-specific Nav2 config files for `allie`, `cornelius`, and `joe`
- a `twist_mux` setup for choosing between Nav2, keyboard, and joystick velocity commands
- direct LaserScan topic remapping for robots that already publish 2D scans

## Suggested simulation workflow

1. Start the robot in Gazebo:

   ```bash
   ros2 launch cororos2_bringup robot_gz.launch.xml robot_model:=<robot_model> rviz:=false
   ```

2. Run Nav2 with SLAM:

   ```bash
   ros2 launch cororos2_navigation cororos2_nav2_slam.launch.xml robot_model:=<robot_model> use_sim_time:=true scan_topic:=/<robot_model>/lidar/scan
   ```

3. Drive manually while SLAM builds the map:

   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -r /cmd_vel:=/key_vel
   ```

   SLAM creates `/map` from lidar scans. The map starts small and grows only where the robot has scanned. Send Nav2 goals only inside the visible map in RViz; goals outside the current map can cause planner `worldToMap failed` errors.

4. Save the map created by SLAM when needed.

   For later SLAM Toolbox localization, save the serialized pose graph:

   ```bash
   mkdir -p ~/maps
   ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '${HOME}/maps/cororos_lab'}"
   ```

   This writes `~/maps/cororos_lab.posegraph`. The `.posegraph` file is what `slam_mode:=localization` needs.

   You can also save a normal Nav2 occupancy map for viewing or other map-server workflows:

   ```bash
   ros2 run nav2_map_server map_saver_cli -f ~/maps/cororos_lab
   ```

5. Relaunch Nav2 with SLAM Toolbox localization:

   ```bash
   ros2 launch cororos2_navigation cororos2_nav2_slam.launch.xml \
     robot_model:=<robot_model> \
     use_sim_time:=true \
     scan_topic:=/<robot_model>/lidar/scan \
     slam_mode:=localization \
     slam_map_file:=${HOME}/maps/cororos_lab
   ```

   > [!NOTE]
   > For hardware navigation, use the same command without `use_sim_time:=true` and keep the default `scan_topic:=/lidar/scan` unless you changed the lidar namespace. `use_sim_time:=true` is only for simulation because it makes ROS use the Gazebo `/clock` topic.

   Keep `slam_mode:=mapping` when building or updating the map.

   If the robot should start from a known pose in the saved graph, also pass:

   ```bash
   slam_map_start_pose:="[x, y, yaw]"
   ```

This package keeps navigation on the live SLAM map.

## Velocity command flow

The navigation launch files publish stamped velocity commands. Start `cororos2_teleop_mux.launch.xml` in a second terminal to run `twist_mux` and optional joystick teleop:

```text
Nav2 -> /cmd_vel_smoothed
/cmd_vel_smoothed -> collision_monitor -> /cmd_vel_nav_safe
keyboard -> /key_vel
joystick -> /joy_vel

/cmd_vel_nav_safe + /key_vel + /joy_vel
-> twist_mux
-> /diff_drive_controller/cmd_vel
```

Nav2, joystick teleop, keyboard teleop, `twist_mux`, and the diff-drive controller are configured to use `geometry_msgs/TwistStamped`.

To skip the external mux and send Nav2 directly to the diff-drive controller, pass:

```bash
use_twist_mux:=false
```

## Keyboard and joystick control

Keyboard teleop is started manually because it needs the active terminal for key presses:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -r /cmd_vel:=/key_vel
```

Start Nav2 in one terminal:

```bash
ros2 launch cororos2_navigation cororos2_nav2_slam.launch.xml \
  robot_model:=joe \
  use_sim_time:=true \
  scan_topic:=/joe/lidar/scan
```

Start the mux and joystick in a second terminal:

```bash
ros2 launch cororos2_navigation cororos2_teleop_mux.launch.xml use_joystick:=true
```

If Nav2 is not running and you only want to test keyboard or joystick driving with robot bringup, start the mux and publish stamped teleop commands:

```bash
ros2 launch cororos2_navigation cororos2_teleop_mux.launch.xml use_joystick:=true
```

Joystick defaults for the Xbox 360 controller:

- device: `/dev/input/js0`
- config: `teleop_twist_joy` package `xbox.config.yaml`
- output topic: `/joy_vel`

Use `joy_dev:=/dev/input/<device>` to choose a different joystick device.

If the joystick device cannot be opened, add your user to the `input` group and then log out and back in:

```bash
sudo usermod -a -G input $USER
```

Joystick controls:

Hold the left trigger button to enable movement. Use the left thumb stick vertical axis for linear movement and the left thumb stick horizontal axis for angular movement. Hold the right trigger button for turbo speed.

The mux priorities are:

- joystick: highest
- keyboard: middle
- Nav2: lowest

This means manual joystick or keyboard commands can override navigation while they are actively publishing.
