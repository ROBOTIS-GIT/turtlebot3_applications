#!/usr/bin/env python3
#
# Copyright 2025 ROBOTIS CO., LTD.
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
#
# Author: ChanHyeong Lee

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    marker_size_arg = DeclareLaunchArgument(
        'marker_size',
        default_value='0.04',
        description='ArUco marker size in meters'
    )
    marker_size = LaunchConfiguration('marker_size')

    aruco_tracker_node = Node(
        package='turtlebot3_aruco_tracker',
        executable='turtlebot3_aruco_tracker',
        name='turtlebot3_aruco_tracker',
        output='screen',
        parameters=[{'marker_size': marker_size}]
    )

    return LaunchDescription([
        marker_size_arg,
        aruco_tracker_node,
    ])
