# cororos2_navigation

This package provides cororos-specific launch files and configs for Nav2 with `slam_toolbox`.

## Package Contents

- `cororos2_nav2_slam.launch.xml`
- `cororos2_teleop_mux.launch.xml`
- robot-specific Nav2 config files for `allie`, `cornelius`, and `joe`
- a `twist_mux` setup for choosing between Nav2, keyboard, and joystick velocity commands
- direct LaserScan topic remapping for robots that already publish 2D scans

## Suggested simulation workflow

1. Start the robot in Gazebo:

   ```bash
   ros2 launch cororos2_bringup robot_gz.launch.xml robot_model:=<robot_model> rviz:=true
   ```

2. Run Nav2 with SLAM:

   ```bash
   ros2 launch cororos2_navigation cororos2_nav2_slam.launch.xml robot_model:=<robot_model> use_sim_time:=true
   ```

3. Start the velocity mux in a second terminal:

   ```bash
   ros2 launch cororos2_navigation cororos2_teleop_mux.launch.xml use_joystick:=false
   ```

4. Drive manually while SLAM builds the map:

   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -r /cmd_vel:=/key_vel
   ```

   In RViz, add a `Map` display and set its topic to `/map` so you can watch SLAM build the map while driving.

   SLAM creates `/map` from lidar scans. The map starts small and grows only where the robot has scanned.

5. Save the map created by SLAM when needed.

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

6. Relaunch Nav2 with SLAM Toolbox localization:

   ```bash
   ros2 launch cororos2_navigation cororos2_nav2_slam.launch.xml \
     robot_model:=<robot_model> \
     use_sim_time:=true \
     use_twist_mux:=false \
     slam_mode:=localization \
     slam_map_file:=${HOME}/maps/cororos_lab
   ```

   If the robot should start from a known pose in the saved graph, also pass:

   ```bash
   slam_map_start_pose:="[x, y, yaw]"
   ```

   > [!NOTE]
   > For hardware navigation, use the same command without `use_sim_time:=true`.
   > Hardware and simulation both use the default `/<robot_model>/lidar/scan` topic.

   In RViz, use the **2D Goal Pose** tool to select the desired goal position on the map.
   Send Nav2 goals only inside the visible map in RViz; goals outside the current map can cause planner `worldToMap failed` errors.

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

To skip the external mux and send Nav2 directly to the diff-drive controller, pass:

```bash
use_twist_mux:=false
```

## Keyboard and joystick control

Keyboard teleop is started manually because it needs the active terminal for key presses:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -r /cmd_vel:=/key_vel
```

For simulation-only manual driving, start Gazebo in one terminal:

```bash
ros2 launch cororos2_bringup robot_gz.launch.xml robot_model:=joe
```

Start the mux and joystick in a second terminal:

```bash
ros2 launch cororos2_navigation cororos2_teleop_mux.launch.xml use_joystick:=true
```

Try moving the joystick or pressing keys in the active teleop keyboard terminal.

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
