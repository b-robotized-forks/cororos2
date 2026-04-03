# Coordinated Robotics

This repository contains the ROS 2 workspace packages and setup for the Coordinated Robotics `cororos2` robots. It currently includes ROS 2 description, bringup, simulation, and sensor integration work for **Allie / Ames**, **Cornelius / Julius**, and **Joe / Jeanine**.

> [!WARNING]
> This repository is under active development. Some robots still contain only partial ROS 2 support, and some hardware integrations are present in launch files but are not yet validated on the real robot.

## Overview

The repository is the ROS 2 port of several robots and currently contains the most complete ROS 2 paths for **Allie / Ames**, **Cornelius / Julius**, and **Joe / Jeanine**.

- **Implemented for Allie and Cornelius:**
  - robot description package
  - bringup package
  - RViz visualization
  - Gazebo Sim integration
  - `diff_drive_controller` setup for ROS 2 simulation
  - simulated lidar, RGB-D camera, IMU, and GPS bridged from Gazebo into ROS
  - integrated launch support for Ouster, RealSense D455, u-blox GPS, and Memsense IMU

- **Implemented for Joe:**
  - robot description package
  - bringup package
  - RViz visualization
  - Gazebo Sim integration
  - `diff_drive_controller` setup
  - simulated lidar, RGB-D camera, IMU, and GPS bridged from Gazebo into ROS
  - integrated launch support for Velodyne VLP-16, RealSense D455, u-blox GPS, Memsense IMU, and an ODrive `ros2_control` hardware interface

- **Robot-specific base backends:**
  - Allie: PWM hardware interface
  - Cornelius: Roboclaw motor driver
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
  - `cororos2_allie_description`
  - `cororos2_allie_bringup`
  - `cororos2_cornelius_description`
  - `cororos2_cornelius_bringup`
  - `cororos2_joe_description`
  - `cororos2_joe_bringup`
  - `memsense_msimu3025_driver`
  - `pwm_hardware_interface`
  - `roboclaw_driver`
  - `odrive_hardware_interface`

## Workspace setup

You can set up the workspace in two ways.

### 1. If you are using RTW

If you have installed RTW from the [RTW installation guide](https://rtw.b-robotized.com/master/tutorials/setting_up_rtw.html#installation-of-rtw), you can create and build the workspace like this:

```bash
rtw workspace create --ws-folder cororos_ws --ros-distro jazzy
rtw ws cororos_ws
rosds
git clone -b ros2 git@github.com:b-robotized-forks/cororos2.git
rosdepi
cb
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

## Starting the robot and the simulation

This repository currently includes launch paths for **Allie / Ames**, **Cornelius / Julius**, and **Joe / Jeanine** in RViz, mock bringup, and Gazebo simulation.

## Allie bringup and simulation

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
  use_memsense:=true
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

## Cornelius bringup and simulation

Cornelius currently supports:

- RViz description launch
- mock bringup with `diff_drive_controller`
- Gazebo simulation with bridged simulated sensors
- optional Roboclaw ROS 2 driver launch path for the real base backend

### 1. View the Cornelius description in RViz

```bash
ros2 launch cororos2_cornelius_description view_cornelius.launch.xml
```

### 2. Start Cornelius bringup with mock hardware

This starts:
- `robot_state_publisher`
- `ros2_control_node`
- `joint_state_broadcaster`
- `diff_drive_controller`
- RViz

```bash
ros2 launch cororos2_cornelius_bringup cornelius.launch.xml
```

You can optionally enable integrated hardware drivers in the same launch:

```bash
ros2 launch cororos2_cornelius_bringup cornelius.launch.xml \
  use_ouster:=true \
  use_realsense:=true \
  use_gps:=true \
  use_memsense:=true
```

### 3. Start Cornelius Gazebo simulation

This starts:
- Gazebo Sim
- the Cornelius URDF
- `robot_state_publisher`
- `gz_ros2_control`
- `joint_state_broadcaster`
- `diff_drive_controller`
- `ros_gz_bridge` for `/clock` and simulated sensors
- optional RViz

```bash
ros2 launch cororos2_cornelius_bringup cornelius_gz.launch.py
```

To run Gazebo without RViz:

```bash
ros2 launch cororos2_cornelius_bringup cornelius_gz.launch.py rviz:=false
```

### 4. Drive the simulated Cornelius robot

```bash
ros2 topic pub /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, twist: {linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}}"
```

You should see the robot move in Gazebo and the odometry change.

### 5. Check the simulated Cornelius ROS sensor topics

The Gazebo launch bridges simulated sensor data into ROS under `/cornelius/...` topics.

You can inspect them with:

```bash
ros2 topic list | grep '^/cornelius/'
```

Examples:

```bash
ros2 topic echo /cornelius/lidar/scan --once
ros2 topic echo /cornelius/rgbd_front/camera_info --once
ros2 topic echo /cornelius/imu/data --once
ros2 topic echo /cornelius/gps/fix --once
```

## Joe / Jeanine bringup and simulation

### 1. View the robot description in RViz

This starts the URDF, `robot_state_publisher`, `joint_state_publisher_gui`, and RViz.

```bash
ros2 launch cororos2_joe_description view_joe.launch.xml
```

### 2. Start Joe bringup with mock hardware

This starts:
- `robot_state_publisher`
- `ros2_control_node` as `b_controlled_box_cm`
- `joint_state_broadcaster`
- `diff_drive_controller`
- RViz

```bash
ros2 launch cororos2_joe_bringup joe.launch.xml
```

You can optionally enable the integrated Joe hardware sensor drivers in the same launch:

```bash
ros2 launch cororos2_joe_bringup joe.launch.xml \
  use_lidar:=true \
  use_realsense:=true \
  use_gps:=true \
  use_memsense:=true
```

### 3. Start Joe Gazebo simulation

This starts:
- Gazebo Sim
- the same Joe URDF used by RViz
- `robot_state_publisher`
- `gz_ros2_control`
- `joint_state_broadcaster`
- `diff_drive_controller`
- `ros_gz_bridge` for `/clock` and simulated sensors
- sensor frame republisher for `/joe/...` topics
- optional RViz

```bash
ros2 launch cororos2_joe_bringup joe_gz.launch.py
```

To run Gazebo without RViz:

```bash
ros2 launch cororos2_joe_bringup joe_gz.launch.py rviz:=false
```

### 4. Drive the simulated robot

In another terminal, publish a velocity command to the Gazebo controller:

```bash
ros2 topic pub /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, twist: {linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}}"
```

You should see the robot move in Gazebo and the odometry change.

### 5. Check the simulated ROS sensor topics

The Gazebo launch bridges simulated sensor data into ROS under `/joe/...` topics.

You can inspect them with:

```bash
ros2 topic list | grep '^/joe/'
```

Examples:

```bash
ros2 topic echo /joe/lidar/scan --once
ros2 topic echo /joe/rgbd_front/camera_info --once
ros2 topic echo /joe/imu/data --once
ros2 topic echo /joe/gps/fix --once
```

## Real hardware sensors bringup

The following hardware drivers are already integrated into `allie.launch.xml`, `cornelius.launch.xml`, and `joe.launch.xml`:

- **Ouster lidar** via `ouster_ros`
- **Velodyne VLP-16** via `velodyne_driver`, `velodyne_pointcloud`, and `velodyne_laserscan`
- **Intel RealSense D455** via `realsense2_camera`
- **u-blox GPS** via `ublox_gps`
- **Memsense IMU** via `memsense_msimu3025_driver`
- **Allie PWM base backend** via `pwm_hardware_interface`
- **Cornelius Roboclaw motor driver** via `roboclaw_driver`
- **Joe ODrive base backend** via `odrive_hardware_interface`

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

### Velodyne VLP-16

Example launch arguments:

```bash
ros2 launch cororos2_joe_bringup joe.launch.xml \
  use_lidar:=true \
  velodyne_device_ip:=<sensor-ip>
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

### PWM base backend

The Allie base can run through the PWM `ros2_control` hardware interface instead of mock hardware.

Example:

```bash
ros2 launch cororos2_allie_bringup allie.launch.xml \
  use_mock_hardware:=false
```

The backend publishes the converted PWM outputs on:

```bash
/allie/pwm
/allie/pwm/front_left
/allie/pwm/rear_left
/allie/pwm/front_right
/allie/pwm/rear_right
```

### Roboclaw motor driver

The current Roboclaw ROS 2 driver is integrated into `cornelius.launch.xml` as an optional real-base backend. It can be launched in no-encoder or encoder-feedback mode.

Example without encoder odometry:

```bash
ros2 launch cororos2_cornelius_bringup cornelius.launch.xml \
  use_mock_hardware:=false \
  use_roboclaw:=true \
  roboclaw_device:=/dev/serial/by-id/<your-device>
```

Example with encoder-based odometry:

```bash
ros2 launch cororos2_cornelius_bringup cornelius.launch.xml \
  use_mock_hardware:=false \
  use_roboclaw:=true \
  roboclaw_use_encoder:=true \
  roboclaw_device:=/dev/serial/by-id/<your-device>
```

> [!WARNING]
> The Roboclaw path is still under active integration. The ROS 2 package, launch wiring, and encoder / no-encoder variants are present, but hardware validation and tuning are still needed.

### ODrive base backend

Joe can also run the real base through the ODrive `ros2_control` hardware interface instead of mock hardware.

Example:

```bash
ros2 launch cororos2_joe_bringup joe.launch.xml \
  use_mock_hardware:=false \
  odrive_front_serial_number:=<front-serial> \
  odrive_rear_serial_number:=<rear-serial>
```

You can combine the ODrive base backend with the integrated sensor drivers in the same launch:

```bash
ros2 launch cororos2_joe_bringup joe.launch.xml \
  use_mock_hardware:=false \
  odrive_front_serial_number:=<front-serial> \
  odrive_rear_serial_number:=<rear-serial> \
  use_lidar:=true \
  use_realsense:=true \
  use_gps:=true \
  use_memsense:=true
```

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
