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
            remappings=[
                ('/camera_info', '/stereo/left/camera_info'),
                ('/image_raw', '/stereo/left/image_rect_color'),
        ]
        ),
        Node(
            package='turtlebot3_automatic_parking_vision',
            executable='turtlebot3_automatic_parking_vision',
            name='turtlebot3_automatic_parking_vision',
            output='screen'),
    ])
