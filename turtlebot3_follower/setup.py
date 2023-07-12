#!/usr/bin/env python3
# Copyright 2023 ROBOTIS CO., LTD.
# Authors: Gilbert

import glob
import os

from setuptools import find_packages
from setuptools import setup

package_name = 'turtlebot3_follower'
share_dir = 'share/' + package_name

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (share_dir, ['package.xml']),
        (share_dir + '/launch', glob.glob(os.path.join('launch', '*.launch.py'))),
        (share_dir + '/config', glob.glob(os.path.join('config', '*'))),
        (share_dir + '/filter', glob.glob(os.path.join('filter', '*.yaml'))),
    ],
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    author=['Gilbert'],
    author_email=['kkjong@robotis.com'],
    maintainer='Will Son',
    maintainer_email='willson@robotis.com',
    keywords=['ROS', 'ROS2', 'examples', 'rclpy'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'Examples of Different TurtleBot3 Usage.'
    ),
    license='Apache License, Version 2.0',
    entry_points={
        'console_scripts': [
            'turtlebot3_follower = \
                turtlebot3_follower.turtlebot3_follower:main',
            'turtlebot3_follower_scan = \
                turtlebot3_follower.laser_subscriber:main',
        ],
    },
)
