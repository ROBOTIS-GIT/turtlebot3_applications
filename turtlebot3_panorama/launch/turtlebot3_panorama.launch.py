# Launch file to initialize and run the TurtleBot3 panorama node.
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
