# Coordinated Robotics

This repository contains the ROS 2 workspace packages and setup for the Coordinated Robotics `cororos2` robots. The current ROS 2 integration work is focused on the **Allie / Ames** platform, including robot description, bringup, Gazebo simulation, sensor integration, and a first PWM (pulse-width modulation) conversion driver for the base.

> [!WARNING]
> This repository is under active development. Some robots still contain only partial ROS 2 support, and some hardware integrations are present in launch files but are not yet validated on the real robot.

## Overview

The repository is the ROS 2 port of several robots. At the moment, the most complete ROS 2 path is for **Allie / Ames**.

- **Implemented for Allie:**
  - robot description package
  - bringup package
  - RViz visualization
  - Gazebo Sim integration
  - `diff_drive_controller` setup
  - simulated lidar, RGB-D camera, IMU, and GPS bridged from Gazebo into ROS
  - integrated launch support for Ouster, RealSense D455, u-blox GPS, Memsense IMU, and PWM conversion driver

- **Allie hardware context:**
  - Ouster OS0-128 lidar
  - Intel RealSense D455 camera
  - u-blox ZED-F9P GPS
  - Memsense MS-IMU3025 IMU
  - REV SPARK MAX motor controller using RC PWM input

- **Current package focus:**
  - `cororos2_allie_description`
  - `cororos2_allie_bringup`
  - `memsense_msimu3025_driver`
  - `allie_pwm_driver`

## Manual workspace setup

To set up the workspace, follow these steps:

### 1. Clone the repository into your ROS 2 workspace

From the root of your workspace (for example `~/cororos2_ws`):

```bash
mkdir -p ~/cororos2_ws/src
cd ~/cororos2_ws/src
git clone -b ros2 git@github.com:b-robotized-forks/cororos2.git cororos2
```

### 2. Install ROS 2 dependencies

```bash
sudo apt update
rosdep update
```

Install package dependencies from the workspace root:

```bash
cd ~/cororos2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Build the workspace

```bash
cd ~/cororos2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Now the workspace is ready for use.

## Starting the robot and the simulation

The main active ROS 2 robot in this repository is **Allie**.

### 1. View the robot description in RViz

This starts the URDF, `robot_state_publisher`, `joint_state_publisher_gui`, and RViz.

```bash
ros2 launch cororos2_allie_description view_allie.launch.xml
```

### 2. Start Allie bringup with mock hardware

This starts:
- `robot_state_publisher`
- `ros2_control_node`
- `joint_state_broadcaster`
- `diff_drive_controller`
- RViz

```bash
ros2 launch cororos2_allie_bringup allie.launch.xml
```

You can optionally enable integrated hardware drivers in the same launch:

```bash
ros2 launch cororos2_allie_bringup allie.launch.xml \
  use_ouster:=true \
  use_realsense:=true \
  use_gps:=true \
  use_memsense:=true \
  use_pwm_driver:=true
```

### 3. Start Allie Gazebo simulation

This starts:
- Gazebo Sim
- the same Allie URDF used by RViz
- `robot_state_publisher`
- `gz_ros2_control`
- `joint_state_broadcaster`
- `diff_drive_controller`
- `ros_gz_bridge` for `/clock` and simulated sensors
- optional RViz

```bash
ros2 launch cororos2_allie_bringup allie_gz.launch.py
```

To run Gazebo without RViz:

```bash
ros2 launch cororos2_allie_bringup allie_gz.launch.py rviz:=false
```

### 4. Drive the simulated robot

In another terminal, publish a velocity command to the Gazebo controller:

```bash
ros2 topic pub /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, twist: {linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}}"
```

You should see the robot move in Gazebo and the odometry change.

### 5. Check the simulated ROS sensor topics

The Gazebo launch bridges simulated sensor data into ROS under `/allie/...` topics.

You can inspect them with:

```bash
ros2 topic list | grep '^/allie/'
```

Examples:

```bash
ros2 topic echo /allie/lidar/scan --once
ros2 topic echo /allie/rgbd_front/camera_info --once
ros2 topic echo /allie/imu/data --once
ros2 topic echo /allie/gps/fix --once
```

## Real hardware sensors bringup

The following hardware drivers are already integrated into `allie.launch.xml`:

- **Ouster lidar** via `ouster_ros`
- **Intel RealSense D455** via `realsense2_camera`
- **u-blox GPS** via `ublox_gps`
- **Memsense IMU** via `memsense_msimu3025_driver`
- **Allie PWM conversion driver** via `allie_pwm_driver`

> [!WARNING]
> The drivers need yet to be tested with real hardware.

### Ouster

Example launch arguments:

```bash
ros2 launch cororos2_allie_bringup allie.launch.xml \
  use_ouster:=true \
  ouster_sensor_hostname:=<sensor-ip> \
  ouster_udp_dest:=<host-ip>
```

### RealSense D455

Example:

```bash
ros2 launch cororos2_allie_bringup allie.launch.xml \
  use_realsense:=true
```

Optionally specify the camera serial number:

```bash
ros2 launch cororos2_allie_bringup allie.launch.xml \
  use_realsense:=true \
  realsense_serial_no:="'<serial>'"
```

### GPS

Example:

```bash
ros2 launch cororos2_allie_bringup allie.launch.xml \
  use_gps:=true \
  gps_device:=/dev/ttyACM0
```

### Memsense IMU

Example:

```bash
ros2 launch cororos2_allie_bringup allie.launch.xml \
  use_memsense:=true \
  memsense_device:=/dev/serial/by-id/<your-device>
```

### PWM conversion driver

The current PWM driver converts ROS velocity commands into left/right PWM values, but it does **not yet** drive a physical hardware output interface.

Example:

```bash
ros2 launch cororos2_allie_bringup allie.launch.xml \
  use_pwm_driver:=true
```

Then test the conversion with:

```bash
ros2 topic echo /allie/pwm
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
```

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
