#!/usr/bin/env python3
# Copyright 2023 ROBOTIS CO., LTD.
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
# Authors: Gilbert, YeonSoo Noh
#
# Description:
#   Launch file to initialize and run the TurtleBot3 panorama node.
#   It sets up required ROS 2 nodes and parameters for panorama operation.

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlebot3_panorama',
            executable='turtlebot3_panorama',
            name='turtlebot3_panorama',
            output='screen'),
    ])
