#!/usr/bin/env python3
# Copyright 2023 ROBOTIS CO., LTD.
# Authors: Gilbert

import glob
import os

from setuptools import find_packages
from setuptools import setup

package_name = 'turtlebot3_automatic_parking'
share_dir = 'share/' + package_name

setup(
    name=package_name,
    version='1.3.3',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (share_dir, ['package.xml']),
        (share_dir + '/launch', glob.glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    author=['Gilbert'],
    author_email=['kkjong@robotis.com'],
    maintainer='Pyo',
    maintainer_email='pyo@robotis.com',
    keywords=['ROS', 'ROS2', 'examples', 'rclpy'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'Package for turtlebot3 automatic_parking.'
    ),
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtlebot3_automatic_parking = \
                turtlebot3_automatic_parking.turtlebot3_automatic_parking:main',
        ],
    },
)
