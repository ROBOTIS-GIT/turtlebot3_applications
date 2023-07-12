#!/usr/bin/env python3
# Copyright 2023 ROBOTIS CO., LTD.
# Authors: Gilbert

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    filter_param = os.path.join(
                get_package_share_directory('turtlebot3_follower'),
                'filter',
                'turtlebot3_follow_filter.yaml')
    return LaunchDescription([
        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='scan_to_scan_filter_chain',
            output='screen',
            parameters=[
                {'params-file': filter_param},
            ],
        ),

        Node(
            package='turtlebot3_follower',
            executable='turtlebot3_follower',
            name='turtlebot3_follower',
            output='screen'),
    ])
