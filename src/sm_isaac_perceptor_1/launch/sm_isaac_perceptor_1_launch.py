# Copyright (c) 2018 Intel Corporation
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

"""This is all-in-one launch script intended for use by nav2 developers."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    xtermprefix = "xterm -xrm 'XTerm*scrollBar:  true' -xrm 'xterm*rightScrollBar: true' -hold -geometry 1000x600 -sl 10000 -e"

    # Get the launch directory
    sm_isaac_perceptor_1_dir = get_package_share_directory("sm_isaac_perceptor_1")
    sm_isaac_perceptor_1_launch_dir = os.path.join(sm_isaac_perceptor_1_dir, "launch")


    sm_isaac_perceptor_1_node = Node(
        package="sm_isaac_perceptor_1",
        executable="sm_isaac_perceptor_1_node",
        name="SmIsaacPerceptor1",
        output="screen",
        # prefix=xtermprefix + " gdb -ex run --args",
        prefix="xterm -hold -e",
        parameters=[
            os.path.join(
                get_package_share_directory("sm_isaac_perceptor_1"),
                "config/sm_isaac_perceptor_1_config.yaml",
            )
        ],
        remappings=[
            # ("/odom", "/odometry/filtered"),
            # ("/sm_isaac_perceptor_1_2/odom_tracker/odom_tracker_path", "/odom_tracker_path"),
            # ("/sm_isaac_perceptor_1_2/odom_tracker/odom_tracker_stacked_path", "/odom_tracker_path_stacked")
        ],
        arguments=["--ros-args", "--log-level", "INFO"],
    )

    keyboard_client_node = Node(
        package="keyboard_client",
        executable="keyboard_server_node.py",
        name="keyboard_client",
        output="screen",
        prefix="xterm -hold -e",
        arguments=["--ros-args", "--log-level", "INFO"],
    )

    # gt_transform_publisher = Node(
    #     package="sm_isaac_perceptor_1",
    #     executable="ue_navigation_frames_ground_truth_adapter.py",
    #     output="screen",
    #     prefix=xtermprefix,
    #     parameters=[{"use_sim_time": use_sim_time}],
    # )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options

    ld.add_action(sm_isaac_perceptor_1_node)
    ld.add_action(keyboard_client_node)

    return ld
