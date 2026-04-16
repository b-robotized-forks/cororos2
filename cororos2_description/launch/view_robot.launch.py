# Copyright (c) 2026, b-robotized Group
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


ROBOT_CONFIG = {
    "allie": {"urdf": "allie.urdf.xacro"},
    "cornelius": {"urdf": "cornelius.urdf.xacro"},
    "joe": {"urdf": "joe.urdf.xacro"},
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

    pkg_description = get_package_share_directory("cororos2_description")
    config = ROBOT_CONFIG[robot_model]
    xacro_file = os.path.join(pkg_description, "urdf", config["urdf"])
    rviz_config = os.path.join(pkg_description, "rviz", "cororos2.rviz")

    robot_description_config = xacro.process_file(
        xacro_file,
        mappings={
            "prefix": LaunchConfiguration("prefix").perform(context),
            "use_mock_hardware": LaunchConfiguration("use_mock_hardware").perform(context),
            "mock_sensor_commands": (
                "true" if _is_true(LaunchConfiguration("use_mock_hardware").perform(context)) else "false"
            ),
        },
    )
    robot_description = {"robot_description": robot_description_config.toxml()}

    actions = [
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description],
        ),
    ]

    if _is_true(LaunchConfiguration("rviz").perform(context)):
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                output="log",
                arguments=["-d", rviz_config],
                additional_env={"LC_ALL": "en_US.UTF-8"},
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_model",
                default_value="allie",
                description="Robot model to view. Choices are: [allie, cornelius, joe].",
                choices=["allie", "cornelius", "joe"],
            ),
            DeclareLaunchArgument(
                "prefix",
                default_value="",
                description="Joint name prefix, useful for multi-robot setups.",
            ),
            DeclareLaunchArgument(
                "use_mock_hardware",
                default_value="true",
                description="Whether to unfold the mock ros2_control configuration.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Launch RViz alongside the robot model.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
