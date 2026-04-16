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
  - Pololu Micro Maestro PWM controller
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
sudo apt install -y python3-pip
rosdep update
export PIP_BREAK_SYSTEM_PACKAGES=1
```

Install package dependencies from the workspace root:

```bash
cd ~/cororos2_ws
rosdep install --from-paths src --ignore-src -r -y
```

The package manifests declare the ROS 2 control, Gazebo, RViz, and hardware-driver dependencies, so `rosdep install` is the supported way to install them. Separate `sudo apt install ros-jazzy-...` commands are not needed.

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
ros2 launch cororos2_description view_robot.launch.xml robot_model:=<robot_model>
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
ros2 launch cororos2_bringup robot_gz.launch.xml robot_model:=<robot_model>
```

To run Gazebo without RViz:

```bash
ros2 launch cororos2_bringup robot_gz.launch.xml robot_model:=<robot_model> rviz:=false
```

### 5. Drive the simulated robot

In another terminal, publish a velocity command to the Gazebo controller:

```bash
ros2 topic pub -r 10 /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, twist: {linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}}"
```

The `diff_drive_controller` has a `cmd_vel_timeout` of 0.5 s, so velocity commands must publish faster than that. Otherwise, the controller may time out between messages and cause intermittent wheel turning.

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

## Hardware bringup notes

The following hardware drivers are already integrated into `cororos2_hw.launch.xml`:

- **Ouster lidar** via `ouster_ros`
- **Velodyne VLP-16** via `velodyne_driver`, `velodyne_pointcloud`, and `velodyne_laserscan`
- **Intel RealSense D455** via `realsense2_camera`
- **u-blox GPS** via `ublox_gps`
- **Memsense IMU** via `memsense_msimu3025_driver`
- **Allie Maestro PWM backend** via `pwm_hardware_interface`
- **Cornelius Roboclaw base backend** via `roboclaw_hardware_interface`
- **Joe ODrive base backend** via `odrive_hardware_interface`

> [!WARNING]
> The drivers still need hardware validation.

### Device permissions

Some hardware backends use serial devices under `/dev/ttyUSB*`, `/dev/ttyACM*`, or `/dev/serial/by-id/...`. On Ubuntu these devices are commonly owned by the `dialout` group. If a hardware launch fails with a permission error while opening a serial device, add your user to `dialout`:

```bash
sudo usermod -a -G dialout $USER
```

Joystick and gamepad devices are commonly exposed through `/dev/input/...`. If joystick input fails with a permission error, add your user to `input`:

```bash
sudo usermod -a -G input $USER
```

Log out and back in, or reboot, before trying again.

### Robot-specific hardware choices

- `robot_model:=allie`
  - base backend: PWM hardware interface
  - lidar stack: Ouster
  - useful extra args:
    - `pwm_device_path:=/dev/serial/by-id/<your-maestro-id>`
    - `pwm_channel_fl:=0 pwm_channel_fr:=1 pwm_channel_rl:=2 pwm_channel_rr:=3`
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

Allie with Maestro PWM base and Ouster:

```bash
ros2 launch cororos2_bringup cororos2_hw.launch.xml robot_model:=allie \
  pwm_device_path:=/dev/serial/by-id/<your-maestro-id> \
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

2. *The base drive returns to neutral while testing.*
   `diff_drive_controller` uses `cmd_vel_timeout: 0.5`, so publish `cmd_vel` faster than that. Otherwise, the controller may time out between messages and cause intermittent wheel turning. For example:
   ```bash
   ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}, angular: {z: 0.5}}"
   ```

3. *Serial devices or joystick input fail with permission errors.*
   Show input devices and their permissions / groups. This is useful to confirm joystick or gamepad devices are owned by `input` or another group:
   ```bash
   ll /dev/input
   ```
   Show whether the `dialout` group exists and which users are in it:
   ```bash
   cat /etc/group | grep dialout
   ```
   Show whether the `input` group exists and which users are in it:
   ```bash
   cat /etc/group | grep input
   ```
   If your user is not listed in the needed group, add it:
   ```bash
   sudo usermod -a -G dialout $USER
   sudo usermod -a -G input $USER
   ```
   Log out and back in, or reboot, before trying again.

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
