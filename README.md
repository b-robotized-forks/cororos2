# Coordinated Robotics

This repository contains the ROS 2 workspace packages and setup for the Coordinated Robotics `cororos2` robots. It currently includes ROS 2 description, bringup, simulation, and sensor integration work for **Allie / Ames** and **Cornelius / Julius**, as well as an Allie PWM conversion driver and a Cornelius Roboclaw motor-driver port.

> [!WARNING]
> This repository is under active development. Some robots still contain only partial ROS 2 support, and some hardware integrations are present in launch files but are not yet validated on the real robot.

## Overview

The repository is the ROS 2 port of several robots and currently contains simulated ROS 2 paths for **Allie / Ames** and **Cornelius / Julius**.

- **Implemented for Allie and Cornelius:**
  - robot description package
  - bringup package
  - RViz visualization
  - Gazebo Sim integration
  - `diff_drive_controller` setup for ROS 2 simulation
  - simulated lidar, RGB-D camera, IMU, and GPS bridged from Gazebo into ROS
  - integrated launch support for Ouster, RealSense D455, u-blox GPS, and Memsense IMU

- **Robot-specific base drivers:**
  - Allie: PWM conversion driver
  - Cornelius: Roboclaw motor driver

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

- **Current package focus:**
  - `cororos2_allie_description`
  - `cororos2_allie_bringup`
  - `cororos2_cornelius_description`
  - `cororos2_cornelius_bringup`
  - `memsense_msimu3025_driver`
  - `allie_pwm_driver`
  - `roboclaw_driver`

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
git clone -b cornelius-integration git@github.com:b-robotized-forks/cororos2.git cororos2
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

This repository currently includes launch paths for **Allie / Ames** and **Cornelius / Julius** in RViz, mock bringup, and Gazebo simulation.

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

## Real hardware sensors bringup

The following hardware drivers are already integrated into `allie.launch.xml` and `cornelius.launch.xml`:

- **Ouster lidar** via `ouster_ros`
- **Intel RealSense D455** via `realsense2_camera`
- **u-blox GPS** via `ublox_gps`
- **Memsense IMU** via `memsense_msimu3025_driver`

The base-driver integration differs by robot:

- **Allie PWM conversion driver** via `allie_pwm_driver`
- **Cornelius Roboclaw motor driver** via `roboclaw_driver`

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
