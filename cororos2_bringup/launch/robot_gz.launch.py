import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    UnsetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


ROBOT_CONFIG = {
    "allie": {
        "controllers": "allie_controllers.yaml",
        "urdf": "allie.urdf.xacro",
        "lidar_sensor": "ouster_lidar",
        "lidar_frame": "ouster_link",
    },
    "cornelius": {
        "controllers": "cornelius_encoder_controllers.yaml",
        "urdf": "cornelius.urdf.xacro",
        "lidar_sensor": "ouster_lidar",
        "lidar_frame": "ouster_link",
    },
    "joe": {
        "controllers": "joe_controllers.yaml",
        "urdf": "joe.urdf.xacro",
        "lidar_sensor": "lidar",
        "lidar_frame": "lidar_link",
    },
}


def _is_true(value: str) -> bool:
    return value.lower() in {"1", "true", "yes", "on"}


def _launch_setup(context, *args, **kwargs):
    del args, kwargs

    robot_model = LaunchConfiguration("robot_model").perform(context)
    if robot_model not in ROBOT_CONFIG:
        raise RuntimeError(
            f"Unsupported robot_model '{robot_model}'. Choose one of: {', '.join(sorted(ROBOT_CONFIG))}."
        )

    config = ROBOT_CONFIG[robot_model]
    pkg_bringup = get_package_share_directory("cororos2_bringup")
    pkg_description = get_package_share_directory("cororos2_description")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    entity_name = LaunchConfiguration("entity_name").perform(context) or robot_model
    xacro_file = os.path.join(pkg_description, "urdf", config["urdf"])
    rviz_config = os.path.join(pkg_description, "rviz", "cororos2.rviz")
    controllers_file = os.path.join(pkg_bringup, "config", config["controllers"])

    robot_description_config = xacro.process_file(
        xacro_file,
        mappings={
            "prefix": "",
            "use_mock_hardware": "false",
            "mock_sensor_commands": "false",
            "sim_gazebo_classic": "false",
            "sim_gazebo": "true",
            "attach_world": "false",
            "simulation_controllers": controllers_file,
        },
    )
    robot_description = {"robot_description": robot_description_config.toxml()}

    lidar_root = f"/world/empty/model/{entity_name}/link/base_footprint/sensor/{config['lidar_sensor']}"
    robot_ns = f"/{robot_model}"

    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
            launch_arguments={"gz_args": LaunchConfiguration("gz_args")}.items(),
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description, {"use_sim_time": True}],
        ),
        Node(
            package="ros_gz_sim",
            executable="create",
            parameters=[
                {
                    "name": entity_name,
                    "topic": "robot_description",
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0,
                }
            ],
            output="screen",
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
            output="screen",
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
                "/navsat@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat",
                f"{lidar_root}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                f"{lidar_root}/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
                f"/world/empty/model/{entity_name}/link/base_footprint/sensor/rgbd_front/image@sensor_msgs/msg/Image[gz.msgs.Image",
                f"/world/empty/model/{entity_name}/link/base_footprint/sensor/rgbd_front/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
                f"/world/empty/model/{entity_name}/link/base_footprint/sensor/rgbd_front/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
                f"/world/empty/model/{entity_name}/link/base_footprint/sensor/rgbd_front/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            ],
            remappings=[
                ("/imu", f"{robot_ns}/raw/imu/data"),
                ("/navsat", f"{robot_ns}/raw/gps/fix"),
                (f"{lidar_root}/scan", f"{robot_ns}/raw/lidar/scan"),
                (f"{lidar_root}/scan/points", f"{robot_ns}/raw/lidar/points"),
                (
                    f"/world/empty/model/{entity_name}/link/base_footprint/sensor/rgbd_front/image",
                    f"{robot_ns}/raw/rgbd_front/image_raw",
                ),
                (
                    f"/world/empty/model/{entity_name}/link/base_footprint/sensor/rgbd_front/depth_image",
                    f"{robot_ns}/raw/rgbd_front/depth_image",
                ),
                (
                    f"/world/empty/model/{entity_name}/link/base_footprint/sensor/rgbd_front/points",
                    f"{robot_ns}/raw/rgbd_front/points",
                ),
                (
                    f"/world/empty/model/{entity_name}/link/base_footprint/sensor/rgbd_front/camera_info",
                    f"{robot_ns}/raw/rgbd_front/camera_info",
                ),
            ],
            output="screen",
        ),
        Node(
            package="cororos2_bringup",
            executable="sim_sensor_frame_republisher",
            output="screen",
            parameters=[
                {
                    "use_sim_time": True,
                    "robot_name": robot_model,
                    "lidar_frame": config["lidar_frame"],
                }
            ],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
            output="screen",
        ),
    ]

    if _is_true(LaunchConfiguration("rviz").perform(context)):
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": True}],
                output="log",
                additional_env={"LC_ALL": "en_US.UTF-8"},
            )
        )

    return actions


def generate_launch_description():
    pkg_bringup = get_package_share_directory("cororos2_bringup")
    pkg_description = get_package_share_directory("cororos2_description")
    floor_world = os.path.join(pkg_bringup, "worlds", "environment.sdf")
    gz_resource_path = os.pathsep.join(
        [
            os.path.dirname(pkg_description),
            os.path.dirname(pkg_bringup),
            os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
        ]
    )
    xdg_config_dirs = os.environ.get(
        "XDG_CONFIG_DIRS_VSCODE_SNAP_ORIG",
        os.environ.get("XDG_CONFIG_DIRS", "/etc/xdg"),
    )
    xdg_data_dirs = os.environ.get(
        "XDG_DATA_DIRS_VSCODE_SNAP_ORIG",
        os.environ.get("XDG_DATA_DIRS", "/usr/local/share:/usr/share"),
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("XDG_CONFIG_DIRS", xdg_config_dirs),
            SetEnvironmentVariable("XDG_DATA_DIRS", xdg_data_dirs),
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", gz_resource_path),
            UnsetEnvironmentVariable("GTK_EXE_PREFIX"),
            UnsetEnvironmentVariable("GTK_IM_MODULE_FILE"),
            UnsetEnvironmentVariable("GTK_MODULES"),
            UnsetEnvironmentVariable("GTK_PATH"),
            UnsetEnvironmentVariable("SNAP"),
            UnsetEnvironmentVariable("SNAP_ARCH"),
            UnsetEnvironmentVariable("SNAP_COMMON"),
            UnsetEnvironmentVariable("SNAP_CONTEXT"),
            UnsetEnvironmentVariable("SNAP_COOKIE"),
            UnsetEnvironmentVariable("SNAP_DATA"),
            UnsetEnvironmentVariable("SNAP_EUID"),
            UnsetEnvironmentVariable("SNAP_INSTANCE_NAME"),
            UnsetEnvironmentVariable("SNAP_LAUNCHER_ARCH_TRIPLET"),
            UnsetEnvironmentVariable("SNAP_LIBRARY_PATH"),
            UnsetEnvironmentVariable("SNAP_NAME"),
            UnsetEnvironmentVariable("SNAP_REAL_HOME"),
            UnsetEnvironmentVariable("SNAP_REVISION"),
            UnsetEnvironmentVariable("SNAP_UID"),
            UnsetEnvironmentVariable("SNAP_USER_COMMON"),
            UnsetEnvironmentVariable("SNAP_USER_DATA"),
            UnsetEnvironmentVariable("SNAP_VERSION"),
            DeclareLaunchArgument(
                "robot_model",
                default_value="allie",
                description="Robot model to spawn in Gazebo. Choices are: [allie, cornelius, joe].",
                choices=["allie", "cornelius", "joe"],
            ),
            DeclareLaunchArgument(
                "gz_args",
                default_value=["-r ", floor_world],
                description='Arguments passed to Gazebo Sim. Use "-r -s empty.sdf" for headless server only.',
            ),
            DeclareLaunchArgument(
                "entity_name",
                default_value="",
                description="Optional Gazebo entity name override. Defaults to the selected robot_model.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Launch RViz alongside Gazebo.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
