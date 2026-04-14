# cororos2_navigation

This package provides cororos-specific launch files, configs, and map assets for Nav2, AMCL, `slam_toolbox`, and `cartographer_ros`.

## What it adds

- `cororos2_nav2_amcl.launch.xml`
- `cororos2_nav2_slam.launch.xml`
- `cororos2_nav2_cartographer.launch.xml`
- `cororos2_teleop_mux.launch.xml`
- robot-specific Nav2 and AMCL config files for `allie`, `cornelius`, and `joe`
- a `twist_mux` setup for choosing between Nav2, keyboard, and joystick velocity commands
- a `cmd_vel_stamper` node that converts the selected `/cmd_vel` `Twist` output into the stamped velocity command expected by the diff-drive controller
- a `scan_republisher` node that normalizes the active robot scan onto `/laser_scan`
- optional `pointcloud_to_laserscan` conversion for robots that use Ouster point clouds

## Suggested simulation workflow

1. Start the robot in Gazebo:

   ```bash
   ros2 launch cororos2_bringup robot_gz.launch.py robot_model:=<robot_model> rviz:=false
   ```

2. Run Nav2 with SLAM:

   ```bash
   ros2 launch cororos2_navigation cororos2_nav2_slam.launch.xml robot_model:=<robot_model> use_sim_time:=true
   ```

3. Drive manually while SLAM builds the map:

   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/key_vel
   ```

   SLAM creates `/map` from lidar scans. The map starts small and grows only where the robot has scanned. Send Nav2 goals only inside the visible map in RViz; goals outside the current map can cause planner `worldToMap failed` errors.

4. Save the map created by SLAM:

   ```bash
   ros2 run nav2_map_server map_saver_cli -f my_map
   ```

5. Stop the SLAM launch and run Nav2 with AMCL against the saved map:

   ```bash
   ros2 launch cororos2_navigation cororos2_nav2_amcl.launch.xml robot_model:=<robot_model> use_sim_time:=true map:=my_map
   ```

Use SLAM when creating a map. Use AMCL when navigating later with a saved map.

## Velocity command flow

The navigation launch files send commands through `/cmd_vel` by default. Start `cororos2_teleop_mux.launch.xml` in a second terminal to run `twist_mux` and optional joystick teleop:

```text
Nav2 -> /cmd_vel_smoothed
/cmd_vel_smoothed -> collision_monitor -> /cmd_vel_nav_checked
keyboard -> /key_vel
joystick -> /joy_vel

/cmd_vel_nav_checked + /key_vel + /joy_vel
-> twist_mux
-> /cmd_vel
-> cmd_vel_stamper
-> /diff_drive_controller/cmd_vel
```

`cmd_vel_stamper` is needed because Nav2, keyboard teleop, and `twist_mux` use `geometry_msgs/Twist`, while the current diff-drive controller command topic expects `geometry_msgs/TwistStamped`.

To skip the external mux and send Nav2 directly through `cmd_vel_stamper`, pass:

```bash
use_twist_mux:=false
```

## Keyboard and joystick control

Keyboard teleop is started manually because it needs the active terminal for key presses:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/key_vel
```

Start Nav2 in one terminal:

```bash
ros2 launch cororos2_navigation cororos2_nav2_slam.launch.xml \
  robot_model:=joe \
  use_sim_time:=true
```

Start the mux and joystick in a second terminal:

```bash
ros2 launch cororos2_navigation cororos2_teleop_mux.launch.xml use_joystick:=true
```

If Nav2 is not running and you only want to test keyboard or joystick driving with robot bringup, also start `cmd_vel_stamper`. It converts `/cmd_vel` from the mux to `/diff_drive_controller/cmd_vel` for the controller:

```bash
ros2 launch cororos2_navigation cororos2_teleop_mux.launch.xml use_joystick:=true use_cmd_vel_stamper:=true
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
