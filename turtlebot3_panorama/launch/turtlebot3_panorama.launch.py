#!/usr/bin/env python3
# Copyright 2023 ROBOTIS CO., LTD.
# Authors: Gilbert

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera_node',
            parameters=[
                {'image_size': [640, 480]},
            ],
        ),
        Node(
            package='turtlebot3_panorama',
            executable='turtlebot3_panorama',
            name='turtlebot3_panorama',
            output='screen'),
    ])
