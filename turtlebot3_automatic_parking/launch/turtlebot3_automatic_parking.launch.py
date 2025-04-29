#!/usr/bin/env python3
# Copyright 2023 ROBOTIS CO., LTD.
# Authors: Gilbert

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlebot3_automatic_parking',
            executable='turtlebot3_automatic_parking',
            name='turtlebot3_automatic_parking',
            output='screen'),
    ])
