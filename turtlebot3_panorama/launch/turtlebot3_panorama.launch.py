#!/usr/bin/env python3
# Copyright 2023 ROBOTIS CO., LTD.
# Licensed under the BSD License

# Authors: Gilbert, YeonSoo Noh

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
