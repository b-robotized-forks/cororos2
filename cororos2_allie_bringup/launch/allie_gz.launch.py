import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    UnsetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_bringup = get_package_share_directory("cororos2_allie_bringup")
    pkg_description = get_package_share_directory("cororos2_allie_description")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    controllers_file = os.path.join(pkg_bringup, "config", "allie_gz_controllers.yaml")
    xacro_file = os.path.join(pkg_description, "urdf", "allie.urdf.xacro")
    rviz_config = os.path.join(pkg_description, "rviz", "allie.rviz")
    floor_world = os.path.join(pkg_bringup, "worlds", "allie_floor.sdf")
    xdg_config_dirs = os.environ.get(
        "XDG_CONFIG_DIRS_VSCODE_SNAP_ORIG",
        os.environ.get("XDG_CONFIG_DIRS", "/etc/xdg"),
    )
    xdg_data_dirs = os.environ.get(
        "XDG_DATA_DIRS_VSCODE_SNAP_ORIG",
        os.environ.get("XDG_DATA_DIRS", "/usr/local/share:/usr/share"),
    )

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

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": LaunchConfiguration("gz_args")}.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        parameters=[
            {
                "name": LaunchConfiguration("entity_name"),
                "topic": "robot_description",
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
            }
        ],
        output="screen",
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        output="log",
        condition=IfCondition(LaunchConfiguration("rviz")),
        additional_env={"LC_ALL": "en_US.UTF-8"},
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("XDG_CONFIG_DIRS", xdg_config_dirs),
            SetEnvironmentVariable("XDG_DATA_DIRS", xdg_data_dirs),
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
                "gz_args",
                default_value=["-r ", floor_world],
                description='Arguments passed to Gazebo Sim. Use "-r -s empty.sdf" for headless server only.',
            ),
            DeclareLaunchArgument(
                "entity_name",
                default_value="allie",
                description="Entity name used when spawning the robot in Gazebo.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Launch RViz alongside Gazebo.",
            ),
            gazebo,
            robot_state_publisher,
            spawn_robot,
            clock_bridge,
            joint_state_broadcaster_spawner,
            diff_drive_controller_spawner,
            rviz,
        ]
    )
