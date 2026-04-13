# Coordinated Robotics

This repository contains the ROS 2 workspace packages and setup for the Coordinated Robotics `cororos2` robots. It currently includes ROS 2 description, bringup, simulation, sensor and drivers integration work for **Allie / Ames**, **Cornelius / Julius**, and **Joe / Jeanine**.

> [!WARNING]
> This repository is under active development. Some robots still contain only partial ROS 2 support, and some hardware integrations are present in launch files but are not yet validated on the real robot.

## Overview

The repository is the ROS 2 port of several robots and currently contains the most complete ROS 2 implementations for **Allie / Ames**, **Cornelius / Julius**, and **Joe / Jeanine**.

- **Robot-specific base backends:**
  - Allie: PWM hardware interface
  - Cornelius: Roboclaw hardware interface
  - Joe: ODrive hardware interface

- **Allie hardware context:**
  - Ouster OS0-128 lidar
  - Intel RealSense D455 camera
  - u-blox ZED-F9P GPS
  - Memsense MS-IMU3025 IMU
  - REV SPARK MAX motor controller using RC PWM input

- **Cornelius hardware context:**
  - Ouster OS0-128 lidar
  - Intel RealSense D455 camera
  - u-blox ZED-F9P GPS
  - Memsense MS-IMU3025 IMU
  - Roboclaw 2x60A motor controller

- **Joe hardware context:**
  - Velodyne VLP-16 lidar
  - Intel RealSense D455 camera
  - u-blox ZED-F9P GPS
  - Memsense MS-IMU3025 IMU
  - custom hoverboard-motor platform driven by ODrive v3.6 motor controllers

- **Current package focus:**
  - `cororos2_description`
  - `cororos2_bringup`
  - `memsense_msimu3025_driver`
  - `pwm_hardware_interface`
  - `roboclaw_hardware_interface`
  - `odrive_hardware_interface`

## Workspace setup

There are two ways to set up the workspace.

### 1. If you are using RTW

If you have installed RTW from the [RTW installation guide](https://rtw.b-robotized.com/master/tutorials/setting_up_rtw.html#installation-of-rtw), you can create and build the workspace in following way:

```bash
rtw workspace create --ws-folder cororos_ws --ros-distro jazzy
rtw ws cororos_ws
rosds
git clone -b ros2 git@github.com:b-robotized-forks/cororos2.git
rosdep_prep
export PIP_BREAK_SYSTEM_PACKAGES=1
rosdepi
cb
```

If GitHub SSH is not configured on your machine yet, you can clone the public repository over HTTPS instead:

```bash
git clone -b ros2 https://github.com/b-robotized-forks/cororos2.git
```

### 2. Manual workspace setup

If you are not using RTW, follow these steps.

#### Clone the repository into your ROS 2 workspace

From the root of your workspace (for example `~/cororos2_ws`):

```bash
mkdir -p ~/cororos2_ws/src
cd ~/cororos2_ws/src
git clone -b ros2 git@github.com:b-robotized-forks/cororos2.git cororos2
```

#### Install ROS 2 dependencies

```bash
sudo apt update
rosdep update
export PIP_BREAK_SYSTEM_PACKAGES=1
```

Install package dependencies from the workspace root:

```bash
cd ~/cororos2_ws
rosdep install --from-paths src --ignore-src -r -y
```

#### Build the workspace

```bash
cd ~/cororos2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Now the workspace is ready for use.

## Starting the robots and the simulation

The launch files use a shared `robot_model` argument:

- `allie`
- `cornelius`
- `joe`

Replace `<robot_model>` in the examples below with the robot you want to use.

The shared bringup can also be launched directly. By default, it starts the selected robot in mock mode.

```bash
ros2 launch cororos2_bringup cororos2_bringup.launch.xml robot_model:=<robot_model>
```

### 1. View the robot description in RViz

This starts the URDF, `robot_state_publisher`, `joint_state_publisher_gui`, and the common RViz config.

```bash
ros2 launch cororos2_description view_robot.launch.py robot_model:=<robot_model>
```

Example RViz views:

| Allie | Cornelius | Joe |
| --- | --- | --- |
| <img src="resources/allie_rviz.png" alt="Allie RViz view" height="220"> | <img src="resources/cornelius_rviz.png" alt="Cornelius RViz view" height="220"> | <img src="resources/joe_rviz.png" alt="Joe RViz view" height="220"> |

### 2. Start mock bringup

This starts:
- `robot_state_publisher`
- `ros2_control_node`
- `joint_state_broadcaster`
- `diff_drive_controller`
- RViz

```bash
ros2 launch cororos2_bringup cororos2_offline.launch.xml robot_model:=<robot_model>
```

### 3. Start hardware bringup

This uses the same bringup, but enables hardware-specific robot description and drivers.

```bash
ros2 launch cororos2_bringup cororos2_hw.launch.xml robot_model:=<robot_model>
```

Common optional hardware toggles:

```bash
ros2 launch cororos2_bringup cororos2_hw.launch.xml robot_model:=<robot_model> \
  use_lidar:=true \
  use_realsense:=true \
  use_gps:=true \
  use_memsense:=true
```

### 4. Start Gazebo simulation

This starts:
- Gazebo Sim
- the selected robot URDF
- `robot_state_publisher`
- `gz_ros2_control`
- `joint_state_broadcaster`
- `diff_drive_controller`
- `ros_gz_bridge` for `/clock` and simulated sensors
- optional RViz

```bash
ros2 launch cororos2_bringup robot_gz.launch.py robot_model:=<robot_model>
```

To run Gazebo without RViz:

```bash
ros2 launch cororos2_bringup robot_gz.launch.py robot_model:=<robot_model> rviz:=false
```

### 5. Drive the simulated robot

In another terminal, publish a velocity command to the Gazebo controller:

```bash
ros2 topic pub -r 10 /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, twist: {linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}}"
```

You should see the robot move in Gazebo and the odometry change.

### 6. Check the simulated ROS sensor topics

The Gazebo launch bridges simulated sensor data into ROS under `/<robot_model>/...` topics.

You can inspect them with:

```bash
ros2 topic list | grep '^/<robot_model>/'
```

Examples:

```bash
ros2 topic echo /<robot_model>/lidar/scan --once
ros2 topic echo /<robot_model>/rgbd_front/camera_info --once
ros2 topic echo /<robot_model>/imu/data --once
ros2 topic echo /<robot_model>/gps/fix --once
```

### 7. Start navigation

Start one of the bringup modes first:

```bash
ros2 launch cororos2_bringup robot_gz.launch.py robot_model:=<robot_model>
```

Then start Nav2 in another terminal. To build a map with SLAM Toolbox:

```bash
ros2 launch cororos2_navigation cororos2_nav2_slam.launch.xml robot_model:=<robot_model> use_sim_time:=true
```

When using SLAM, the map starts small and grows only where the robot has scanned with the lidar. Drive the robot manually first:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/key_vel
```

Wait until `/map` covers the area around the robot, and only send Nav2 goals inside the visible map in RViz. If a goal is outside the current map, the planner can report `worldToMap failed` because it cannot plan outside the known costmap.

Save the completed map:

```bash
ros2 run nav2_map_server map_saver_cli -f my_map
```

To localize on an existing map with AMCL:

```bash
ros2 launch cororos2_navigation cororos2_nav2_amcl.launch.xml robot_model:=<robot_model> use_sim_time:=true map:=my_map
```

Use SLAM when creating a map. Use AMCL when navigating later with a saved map.

The navigation launch routes commands through `twist_mux` by default:

```text
Nav2 -> /cmd_vel_smoothed
/cmd_vel_smoothed -> collision_monitor -> /cmd_vel_nav_checked
keyboard -> /key_vel
joystick -> /joy_vel
/cmd_vel_nav_checked + /key_vel + /joy_vel -> twist_mux -> /cmd_vel
cmd_vel_stamper -> /diff_drive_controller/cmd_vel
```

Keyboard teleop can be started separately with:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/key_vel
```

Joystick teleop can be started with the navigation launch:

```bash
ros2 launch cororos2_navigation cororos2_nav2_slam.launch.xml robot_model:=<robot_model> use_sim_time:=true use_joystick:=true
```

## Hardware bringup notes

The following hardware drivers are already integrated into `cororos2_hw.launch.xml`:

- **Ouster lidar** via `ouster_ros`
- **Velodyne VLP-16** via `velodyne_driver`, `velodyne_pointcloud`, and `velodyne_laserscan`
- **Intel RealSense D455** via `realsense2_camera`
- **u-blox GPS** via `ublox_gps`
- **Memsense IMU** via `memsense_msimu3025_driver`
- **Allie PWM base backend** via `pwm_hardware_interface`
- **Cornelius Roboclaw base backend** via `roboclaw_hardware_interface`
- **Joe ODrive base backend** via `odrive_hardware_interface`

> [!WARNING]
> The drivers still need hardware validation.

### Robot-specific hardware choices

- `robot_model:=allie`
  - base backend: PWM hardware interface
  - lidar stack: Ouster
  - useful extra args:
    - `ouster_sensor_hostname:=<sensor-ip>`
    - `ouster_udp_dest:=<host-ip>`

- `robot_model:=cornelius`
  - base backend: Roboclaw hardware interface
  - lidar stack: Ouster
  - useful extra args:
    - `roboclaw_device:=/dev/serial/by-id/<your-device>`
    - `roboclaw_use_encoder:=true`
    - `ouster_sensor_hostname:=<sensor-ip>`
    - `ouster_udp_dest:=<host-ip>`

- `robot_model:=joe`
  - base backend: ODrive hardware interface
  - lidar stack: Velodyne VLP-16
  - useful extra args:
    - `odrive_front_serial_number:=<front-serial>`
    - `odrive_rear_serial_number:=<rear-serial>`
    - `velodyne_device_ip:=<sensor-ip>`

### Hardware examples

Allie with PWM base and Ouster:

```bash
ros2 launch cororos2_bringup cororos2_hw.launch.xml robot_model:=allie \
  ouster_sensor_hostname:=<sensor-ip> \
  ouster_udp_dest:=<host-ip>
```

Cornelius with Roboclaw base hardware interface:

```bash
ros2 launch cororos2_bringup cororos2_hw.launch.xml robot_model:=cornelius \
  roboclaw_device:=/dev/serial/by-id/<your-device>
```

Cornelius with encoder-based Roboclaw odometry:

```bash
ros2 launch cororos2_bringup cororos2_hw.launch.xml robot_model:=cornelius \
  roboclaw_use_encoder:=true \
  roboclaw_device:=/dev/serial/by-id/<your-device>
```

Joe with ODrive base and Velodyne:

```bash
ros2 launch cororos2_bringup cororos2_hw.launch.xml robot_model:=joe \
  odrive_front_serial_number:=<front-serial> \
  odrive_rear_serial_number:=<rear-serial> \
  velodyne_device_ip:=<sensor-ip>
```

RealSense on any robot:

```bash
ros2 launch cororos2_bringup cororos2_hw.launch.xml robot_model:=<robot_model> \
  use_realsense:=true \
  realsense_serial_no:="'<serial>'"
```

GPS on any robot:

```bash
ros2 launch cororos2_bringup cororos2_hw.launch.xml robot_model:=<robot_model> \
  use_gps:=true \
  gps_device:=/dev/ttyACM0
```

Memsense IMU on any robot:

```bash
ros2 launch cororos2_bringup cororos2_hw.launch.xml robot_model:=<robot_model> \
  use_memsense:=true \
  memsense_device:=/dev/serial/by-id/<your-device>
```

PWM output topics for Allie:

```bash
/allie/pwm
/allie/pwm/front_left
/allie/pwm/rear_left
/allie/pwm/front_right
/allie/pwm/rear_right
```

> [!WARNING]
> The Roboclaw path is still under active integration. The ROS 2 package, launch wiring, and encoder / no-encoder variants are present, but hardware validation and tuning are still needed.

> [!NOTE]
> The ODrive backend helper uses the Python `odrive` module. If you install workspace dependencies with `rosdep`, it is pulled in through the `python3-odrive-pip` rosdep key. Otherwise install it manually with `python3 -m pip install --upgrade odrive`.

## Troubleshooting

1. *Controllers are already loaded or active.*
   This usually means stale nodes from a previous launch are still running.
   ```bash
   pkill -f ros2_control_node
   pkill -f controller_manager/spawner
   pkill -f robot_state_publisher
   ```
   Then relaunch once from a clean terminal.

2. *PWM output keeps returning to neutral while testing.*
   The PWM driver uses a timeout of `0.5 s`. Publish `cmd_vel` faster than that, for example:
   ```bash
   ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}, angular: {z: 0.5}}"
   ```

## Roboclaw Diagnostics And Telemetry

Cornelius's `roboclaw_hardware_interface` publishes `/diagnostics` with one Roboclaw diagnostic entry. The diagnostic message reports conditions such as overcurrent, emergency stop, battery voltage faults, driver faults, and temperature faults.

The Roboclaw diagnostic includes:

- `device`
- `address`
- `main_battery_voltage_v`
- `logic_battery_voltage_v`
- `m1_current_a`
- `m2_current_a`
- `temp1_c`
- `temp2_c`
- `error_word`

Published telemetry topics:

- `roboclaw/battery_state`
- `roboclaw/logic_battery_voltage`
- `roboclaw/m1_current`
- `roboclaw/m2_current`
- `roboclaw/temp1`
- `roboclaw/temp2`
