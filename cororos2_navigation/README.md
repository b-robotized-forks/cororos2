# cororos2_navigation

This package provides cororos-specific launch files and configs for Nav2 with `slam_toolbox`.

## Package Contents

- `cororos2_nav2_slam.launch.xml`
- `cororos2_teleop_mux.launch.xml`
- robot-specific Nav2 config files for `allie`, `cornelius`, and `joe`
- a `twist_mux` setup for choosing between Nav2, keyboard, and joystick velocity commands
- direct LaserScan topic remapping for robots that already publish 2D scans

## Suggested Workflows

### Gazebo Simulation Workflow

1. Start the robot in Gazebo:

   ```bash
   ros2 launch cororos2_bringup robot_gz.launch.xml robot_model:=<robot_model>
   ```

2. Run Nav2 with SLAM:

   ```bash
   ros2 launch cororos2_navigation cororos2_nav2_slam.launch.xml robot_model:=<robot_model> use_sim_time:=true
   ```

   In RViz, open `Global Options` and set `Fixed Frame` to `map`.
   Add a `Map` display and set its topic to `/map` so you can watch SLAM build the map while driving.

3. Start the velocity mux in a second terminal:

   ```bash
   ros2 launch cororos2_navigation cororos2_teleop_mux.launch.xml use_joystick:=false
   ```

4. Drive manually while SLAM builds the map:

   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -r /cmd_vel:=/key_vel
   ```

### Hardware Workflow

1. Start hardware bringup:

   ```bash
   ros2 launch cororos2_bringup cororos2_hw.launch.xml robot_model:=<robot_model>
   ```

   This starts the base, robot state publisher, fused odometry by default, and the configured sensor stack for the selected robot.

2. Run Nav2 with SLAM:

   ```bash
   ros2 launch cororos2_navigation cororos2_nav2_slam.launch.xml robot_model:=<robot_model>
   ```

   In RViz, open `Global Options` and set `Fixed Frame` to `map`.
   Add a `Map` display and set its topic to `/map` so you can watch SLAM build the map while driving.

3. Start the velocity mux in a second terminal and drive with joystick:

   ```bash
   ros2 launch cororos2_navigation cororos2_teleop_mux.launch.xml use_joystick:=true
   ```

4. OR drive with keyboard:

   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -r /cmd_vel:=/key_vel
   ```

### Saving The Map

1. Save the serialized pose graph for later SLAM Toolbox localization:

   ```bash
   mkdir -p ~/maps
   ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '${HOME}/maps/cororos_lab'}"
   ```

   This writes `~/maps/cororos_lab.posegraph`. The `.posegraph` file is what `slam_mode:=localization` needs.

2. Optionally save a normal Nav2 occupancy map for viewing or other map-server workflows:

   ```bash
   ros2 run nav2_map_server map_saver_cli -f ~/maps/cororos_lab
   ```

### Localization Workflow

- Gazebo:

Start Gazebo in one terminal and in another terminal do:

```bash
ros2 launch cororos2_navigation cororos2_nav2_slam.launch.xml \
  robot_model:=<robot_model> \
  use_sim_time:=true \
  use_twist_mux:=false \
  slam_mode:=localization \
  slam_map_file:=${HOME}/maps/cororos_lab
```

- Hardware:
Start hardware in one terminal and in separate terminal do:

```bash
ros2 launch cororos2_navigation cororos2_nav2_slam.launch.xml \
  robot_model:=<robot_model> \
  use_sim_time:=false \
  use_twist_mux:=false \
  slam_mode:=localization \
  slam_map_file:=${HOME}/maps/cororos_lab
```

If the robot should start from a known pose in the saved graph, also pass:

```bash
slam_map_start_pose:="[x, y, yaw]"
```

In RViz, use the **2D Goal Pose** tool to select the desired goal position on the map.
Send Nav2 goals only inside the visible map in RViz; goals outside the current map can cause planner `worldToMap failed` errors.

## Fused odometry

With fused odometry enabled, `robot_localization` fuses `/diff_drive_controller/odom` and `/<robot_model>/imu/data` to publish odometry.

Common commands:

- Hardware with disabled fused odometry:

```bash
ros2 launch cororos2_bringup cororos2_hw.launch.xml robot_model:=<robot_model> use_fused_odometry:=false
```

- Gazebo with disabled fused odometry:

```bash
ros2 launch cororos2_bringup robot_gz.launch.xml robot_model:=<robot_model> use_fused_odometry:=false
```

Defaults:

- Hardware: `use_fused_odometry:=true`
- Gazebo: `use_fused_odometry:=true`
- Mock/offline: `use_fused_odometry:=false`

Useful checks:

```bash
ros2 topic echo /odometry/filtered --once
ros2 topic echo /<robot_model>/imu/data --once
ros2 run tf2_ros tf2_echo odom base_footprint
```

## Keyboard and joystick control

Keyboard teleop is started manually because it needs the active terminal for key presses:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -r /cmd_vel:=/key_vel
```

For simulation-only manual driving, start Gazebo in one terminal:

```bash
ros2 launch cororos2_bringup robot_gz.launch.xml robot_model:=<robot_model>
```

Start the mux and joystick in a second terminal:

```bash
ros2 launch cororos2_navigation cororos2_teleop_mux.launch.xml use_joystick:=true
```

Joystick defaults for the Xbox 360 controller.
Use `joy_dev:=/dev/input/<device>` to choose a different joystick device.

Try moving the joystick or pressing keys in the active teleop keyboard terminal:

Hold the left trigger button to enable movement. Use the left thumb stick vertical axis for linear movement and the left thumb stick horizontal axis for angular movement. Hold the right trigger button for turbo speed.

> [!NOTE]
> If the joystick device cannot be opened, add your user to the `input` group and then log out and back in:
>
> ```bash
> sudo usermod -a -G input $USER
> ```


> [!NOTE]
> To skip the external mux and send Nav2 directly to the diff-drive controller, pass:
>
> ```bash
> use_twist_mux:=false
> ```
