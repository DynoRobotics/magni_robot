# Copyright 2020 ros2_control Development Team
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

import argparse

import launch
import launch_ros

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description(args=None):

    arg_show_rviz = DeclareLaunchArgument(
        "start_rviz",
        default_value="false",
        description="start RViz automatically with the launch file",
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("magni_description"), "urdf", "magni_system.urdf.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    magni_diff_drive_controller = PathJoinSubstitution(
        [
            FindPackageShare("magni_bringup"),
            "param",
            "base.yaml",
        ]
    )

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    launch_prefix = []

    if (args != None and args.debug_controller_manager == True):
        launch_prefix = ['gdbserver :9091']
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, magni_diff_drive_controller],
        prefix=launch_prefix,
        # arguments=['--ros-args', '--log-level', 'DEBUG'],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    spawn_diff_drive_controller = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["magni_base_controller"],
        output="screen",
    )

    spawn_joint_state_broadcaster_controller = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    spawn_motor_diagnostics_broadcaster_controller = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["motor_diagnostics_broadcaster"],
        output="screen",
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("magni_description"), "config", "diffbot.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(LaunchConfiguration("start_rviz")),
    )

    return LaunchDescription(
        [
            arg_show_rviz,
            node_robot_state_publisher,
            controller_manager_node,
            spawn_diff_drive_controller,
            spawn_joint_state_broadcaster_controller,
            spawn_motor_diagnostics_broadcaster_controller,
            rviz_node,
        ]
    )

def add_parser_arguments(parser):

    parser.add_argument(
        '--debug-controller-manager', action='store_true', default=False)

def launch_main(argv=None):
    """Run launch for the magni."""

    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    add_parser_arguments(parser)

    if argv is None:
        args = parser.parse_args()
    else:
        args = parser.parse_args(argv)
    
    launch_service = launch.LaunchService(debug=False)

    launch_service.include_launch_description(generate_launch_description(args))

    return launch_service.run()

if __name__ == '__main__':
    launch_main()
