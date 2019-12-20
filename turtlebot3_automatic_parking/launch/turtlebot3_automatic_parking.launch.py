#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
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
# Authors: Ryan Shim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    rviz_config_dir = os.path.join(
        get_package_share_directory('turtlebot3_automatic_parking'),
        'rviz',
        'turtlebot3_automatic_parking.rviz')
    urdf_file = os.path.join(get_package_share_directory(
        'turtlebot3_description'),
        'urdf',
        'turtlebot3_burger.urdf')

    return LaunchDescription([
        Node(
            package='turtlebot3_automatic_parking',
            node_executable='turtlebot3_automatic_parking',
            node_name='turtlebot3_automatic_parking',
            output='screen'),

        Node(
            package='rviz2',
            node_executable='rviz2',
            node_name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen'),

        Node(
            package='robot_state_publisher',
            node_executable='robot_state_publisher',
            node_name='robot_state_publisher',
            arguments=[urdf_file],
            output='screen')            
    ])
