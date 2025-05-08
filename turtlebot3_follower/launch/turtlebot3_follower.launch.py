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
# Authors: Hyungyu Kim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    number_of_follower = 3
    follower = Node(
        package='turtlebot3_follower',
        executable='follower',
        output='screen',
        arguments=[str(number_of_follower)],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    nodes = []
    nodes.append(follower)
    for i in range(number_of_follower):
        namespace = f'TB3_{i+2}'

        ctrl_yaml_path = os.path.join(
            get_package_share_directory('turtlebot3_follower'),
            'param',
            f'controll_server{i+1}.yaml'
        )

        ctrl_node = Node(
            package='nav2_controller',
            executable='controller_server',
            namespace=namespace,
            name='controller_server',
            output='screen',
            parameters=[
                ctrl_yaml_path,
                {'use_sim_time': use_sim_time}],
            remappings=[
                (f'/{namespace}/cmd_vel', f'/{namespace}/cmd_vel_not_smoothed')]
        )

        lifecycle_node = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            namespace=namespace,
            name='lifecycle_manager_controller',
            output='screen',
            parameters=[
                {'autostart': True},
                {'use_sim_time': use_sim_time},
                {'node_names': ['controller_server', 'velocity_smoother']}],
        )

        velocity_smoother_node = Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            namespace=namespace,
            name='velocity_smoother',
            output='screen',
            parameters=[ctrl_yaml_path],
            remappings=[
                (f'/{namespace}/cmd_vel', f'/{namespace}/cmd_vel_not_smoothed'),
                (f'/{namespace}/cmd_vel_smoothed', f'/{namespace}/cmd_vel')]
        )
        nodes.append(ctrl_node)
        nodes.append(lifecycle_node)
        nodes.append(velocity_smoother_node)

    return LaunchDescription(nodes)
