#!/usr/bin/env python3
#################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
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
#################################################################################

# Authors: Gilbert #

import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

import pickle


class laserSubscriber(Node):
    def __init__(self):
        super().__init__('laser_subscriber')

        package_dir = get_package_share_directory('turtlebot3_follower')
        self.config_dir = os.path.join(package_dir, 'config')
        self.laser_scan_data =[]
        self.comments=[]
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan_filtered',
            self._scan_callback,
        qos_profile=qos_profile_sensor_data)

        self.run_timer = self.create_timer(0.05, self._subscriber)

    def _scan_callback(self, msg):
        normalized_range = []
        normalized_intensity = []
        normalized_index = []
        standardized_scan_length = 360
        normalized_rate = standardized_scan_length / len(msg.ranges)

        for i in range(len(msg.ranges)):
            normalized_index.append(round(i * normalized_rate))
        range_index = 0
        for i in range(standardized_scan_length):
            if i in normalized_index:
                normalized_range.append(msg.ranges[range_index])
                normalized_intensity.append(msg.intensities[range_index])
                range_index += 1
            else:
                normalized_range.append(float('nan'))
                normalized_intensity.append(float('nan'))

        self.scan.ranges = normalized_range
        self.scan.intensities = normalized_intensity

        self.is_scan_received = True

    def _subscriber(self):
        comment = input('Right a comment and pres Enter to continue..\n')
        self.comments.append(comment)
        if self.is_scan_received:
            self.laser_scan_data.append(self.msg)
            with open(self.config_dir + '/add_comment', 'wb') as file:
                pickle.dump(self.comments, file)
            with open(self.config_dir + '/add_data', 'wb') as file:
                pickle.dump(self.laser_scan_data, file)


def main(args=None):
    rclpy.init(args=args)
    laser_subscriber = laserSubscriber()

    try:
        rclpy.spin(laser_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        laser_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
