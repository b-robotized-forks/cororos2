# cororos2_navigation

This package provides cororos-specific launch files, configs, and map assets for Nav2, AMCL, `slam_toolbox`, and `cartographer_ros`.

## What it adds

- `cororos2_nav2_amcl.launch.xml`
- `cororos2_nav2_slam.launch.xml`
- `cororos2_nav2_cartographer.launch.xml`
- robot-specific Nav2 and AMCL config files for `allie`, `cornelius`, and `joe`
- a `twist_mux` setup for choosing between Nav2, keyboard, and joystick velocity commands
- a `cmd_vel_stamper` node that converts the selected `/cmd_vel` `Twist` output into the stamped velocity command expected by the diff-drive controller
- a `scan_republisher` node that normalizes the active robot scan onto `/laser_scan`
- optional `pointcloud_to_laserscan` conversion for robots that use Ouster point clouds

## Suggested simulation workflow

1. Start the robot in Gazebo:

   ```bash
   ros2 launch cororos2_bringup robot_gz.launch.py robot_model:=cornelius rviz:=false
   ```

2. Run Nav2 with SLAM:

   ```bash
   ros2 launch cororos2_navigation cororos2_nav2_slam.launch.xml robot_model:=cornelius use_sim_time:=true
   ```

3. Save the map created by SLAM:

   ```bash
   ros2 run nav2_map_server map_saver_cli -f my_map
   ```

4. Or run Nav2 with AMCL against a saved map:

   ```bash
   ros2 launch cororos2_navigation cororos2_nav2_amcl.launch.xml robot_model:=cornelius use_sim_time:=true map:=my_map
   ```

   Use `placeholder_map` only as a temporary dummy map before a real map is saved.

## Velocity command flow

The navigation launch files start `twist_mux` by default:

```text
Nav2 -> /cmd_vel_smoothed
keyboard -> /key_vel
joystick -> /joy_vel

/cmd_vel_smoothed + /key_vel + /joy_vel
-> twist_mux
-> /cmd_vel
-> cmd_vel_stamper
-> /diff_drive_controller/cmd_vel
```

`cmd_vel_stamper` is needed because Nav2, keyboard teleop, and `twist_mux` use `geometry_msgs/Twist`, while the current diff-drive controller command topic expects `geometry_msgs/TwistStamped`.

To disable the mux and send Nav2 directly through `cmd_vel_stamper`, pass:

```bash
use_twist_mux:=false
```

## Keyboard and joystick control

Keyboard teleop is started manually because it needs the active terminal for key presses:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/key_vel
```

Joystick support can be started with the navigation launch:

```bash
ros2 launch cororos2_navigation cororos2_nav2_slam.launch.xml \
  robot_model:=joe \
  use_sim_time:=true \
  use_joystick:=true
```

Joystick defaults:

- device: `/dev/input/js0`
- config: `teleop_twist_joy_logitech_f710_diff_drive.config.yaml`
- output topic: `/joy_vel`

The mux priorities are:

- joystick: highest
- keyboard: middle
- Nav2: lowest

This means manual joystick or keyboard commands can override navigation while they are actively publishing.
