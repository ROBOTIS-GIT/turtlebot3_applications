#!/usr/bin/env python3
#################################################################################
# Copyright 2023 ROBOTIS CO., LTD.
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

from math import sin, cos, pi
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty

import numpy as np
from transforms3d.euler import quat2euler


class AutomaticParking(Node):

    def __init__(self):
        super().__init__('automatic_parking')

        # Set initial value
        self.scan = LaserScan()
        self.odom = Odometry()
        self.euler = [0.0, 0.0, 0.0]
        self.center_angle = 0
        self.start_angle = 0
        self.end_angle = 0
        self.start_point = [0.0, 0.0]
        self.end_point = [0.0, 0.0]
        self.center_point = [0.0, 0.0]
        self.rotation_point = [0.0, 0.0]
        self.parking_sequence = 0
        self.theta = 0.0

        # Set publisher
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            qos_profile=QoSProfile(depth=10))

        self.reset_publisher = self.create_publisher(
            Empty,
            '/reset',
            qos_profile=QoSProfile(depth=10))

        self.scan_spot_publisher = self.create_publisher(
            LaserScan,
            '/scan_spot',
            qos_profile=QoSProfile(depth=10))

        # Set subscriber
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self._scan_callback,
            qos_profile=QoSProfile(depth=10))

        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self._odom_callback,
            qos_profile=QoSProfile(depth=10))

        self._run_timer = self.create_timer(0.05, self._run)

    def _scan_callback(self, msg):
        self.scan = msg
        scan_spot = LaserScan()
        scan_spot_list = list(msg.intensities)
        for i in range(360):
            scan_spot_list[i] = 0
        scan_spot_list[self.start_angle] = msg.ranges[self.start_angle] + 10000
        scan_spot_list[self.center_angle] = msg.ranges[self.center_angle] + 10000
        scan_spot_list[self.end_angle] = msg.ranges[self.end_angle] + 10000
        scan_spot.intensities = tuple(scan_spot_list)
        self.scan_spot_publisher.publish(scan_spot)
        self.get_logger().info('scan {0}'.format(self.scan))

    def _odom_callback(self, msg):
        self.odom = msg
        orientation = msg.pose.pose.orientation
        quaternion = (
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        )

        self.euler = quat2euler(quaternion)
        self.get_logger().info('odom {0}'.format(self.odom))

    def _get_point(self, start_angle_distance):
        angle = start_angle_distance[0]
        angle = np.deg2rad(angle - 180)
        distance = start_angle_distance[1]

        if angle >= 0 and angle < pi / 2:
            x = distance * cos(angle) * -1
            y = distance * sin(angle) * -1
        elif angle >= pi / 2 and angle < pi:
            x = distance * cos(angle) * -1
            y = distance * sin(angle) * -1
        elif angle >= -pi / 2 and angle < 0:
            x = distance * cos(angle) * -1
            y = distance * sin(angle) * -1
        else:
            x = distance * cos(angle) * -1
            y = distance * sin(angle) * -1

        return [x, y]

    def _get_angle_distance(self, angle):
        distance = self.scan.ranges[int(angle)]
        if self.scan.ranges[int(angle)] != None and distance != 0:
            angle = int(angle)
            distance = distance
        return angle, distance

    def _scan_parking_spot(self):
        intensity_index = []
        index_count = []
        spot_angle_index = []
        min_scan_angle = 30
        max_scan_angle = 330
        intensity_threshold = 100

        try:
            for i in range(360):
                if i >= min_scan_angle and i < max_scan_angle:
                    spot_intensity = self.scan.intensities[i] ** 2 * self.scan.ranges[i] / 100000
                    if spot_intensity >= intensity_threshold:
                        intensity_index.append(i)
                        index_count.append(i)
                    else:
                        intensity_index.append(0)
                else:
                    intensity_index.append(0)

            for i in index_count:
                if abs(i - index_count[int(len(index_count) / 2)]) < 20:
                    spot_angle_index.append(i)
                    if len(spot_angle_index) > 10:
                        scan_done = True
                        self.center_angle = spot_angle_index[int(len(spot_angle_index) / 2)]
                        self.start_angle = spot_angle_index[2]
                        self.end_angle = spot_angle_index[-3]
                    else:
                        scan_done = False
        except:
            scan_done = False

        return scan_done

    def _finding_spot_position(self):
        self.get_logger().info("scan parking spot done!")
        start_angle_distance = self._get_angle_distance(self.start_angle)
        center_angle_distance = self._get_angle_distance(self.center_angle)
        end_angle_distance = self._get_angle_distance(self.end_angle)

        if start_angle_distance[1] != 0 and center_angle_distance[1] != 0 and end_angle_distance[1] != 0:
            self.get_logger().info("calibration......")
            self.start_point = self._get_point(start_angle_distance)
            self.center_point = self._get_point(center_angle_distance)
            self.end_point = self._get_point(end_angle_distance)
            self.theta = np.arctan2(
                self.start_point[1] - self.end_point[1],
                self.start_point[0] - self.end_point[0])
            return True
        else:
            self.get_logger().warn("wrong scan!!")
            return False

    def _print_parking_log(self):
        self.get_logger().info("=================================")
        self.get_logger().info("|        |     x     |     y     |")
        self.get_logger().info('| start  | {0:>10.3f}| {1:>10.3f}|'.format(self.start_point[0], self.start_point[1]))
        self.get_logger().info('| center | {0:>10.3f}| {1:>10.3f}|'.format(self.center_point[0], self.center_point[1]))
        self.get_logger().info('| end    | {0:>10.3f}| {1:>10.3f}|'.format(self.end_point[0], self.end_point[1]))
        self.get_logger().info("=================================")
        self.get_logger().info('| theta  | {0:.2f} deg'.format(np.rad2deg(self.theta)))
        self.get_logger().info('| yaw    | {0:.2f} deg'.format(np.rad2deg(self.euler[2])))
        self.get_logger().info("=================================")
        self.get_logger().info("===== Go to parking spot!!! =====")

    def _rotate_origin_only(self, radians):
        self.rotation_point[0] = self.center_point[0] * cos(-(pi / 2 - radians)) \
            + self.center_point[1] * sin(-(pi / 2 - radians))
        self.rotation_point[1] = -self.center_point[0] * sin(-(pi / 2 - radians)) \
            + self.center_point[1] * cos(-(pi / 2 - radians))

    def _stop_and_reset(self):
        cmd_vel = Twist()
        reset = Empty()
        cmd_vel.linear.x = 0
        cmd_vel.angular.z = 0
        self.cmd_vel_publisher.publish(cmd_vel)
        time.sleep(1)
        self.reset_publisher.publish(reset)
        time.sleep(3)

    def _run(self):
        cmd_vel = Twist()
        if self.parking_sequence == 0:
            if self._scan_parking_spot():
                if self._finding_spot_position():
                    self._print_parking_log()
                    self.parking_sequence = 1
            # else:
            #     self.get_logger().error("Fail finding parking spot.")

        elif self.parking_sequence == 1:
            init_yaw = self.euler[2]
            self.yaw = self.theta + self.euler[2]
            if self.theta > 0:
                if self.theta - init_yaw > 0.1:
                    cmd_vel.linear.x = 0.0
                    cmd_vel.angular.z = 0.2
                else:
                    self._stop_and_reset()
                    self._rotate_origin_only(init_yaw)
                    self.parking_sequence = 2
            else:
                if self.theta - init_yaw < -0.1:
                    cmd_vel.linear.x = 0.0
                    cmd_vel.angular.z = -0.2
                else:
                    self._stop_and_reset()
                    self._rotate_origin_only(init_yaw)
                    self.parking_sequence = 2

        elif self.parking_sequence == 2:
            if abs(self.odom.pose.pose.position.x - (self.rotation_point[1])) > 0.02:
                if self.odom.pose.pose.position.x > (self.rotation_point[1]):
                    cmd_vel.linear.x = -0.05
                    cmd_vel.angular.z = 0.0
                else:
                    cmd_vel.linear.x = 0.05
                    cmd_vel.angular.z = 0.0
            else:
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.0
                self.parking_sequence = 3

        elif self.parking_sequence == 3:
            if self.theta + self.euler[2] > -pi / 2:
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = -0.2
            else:
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.0
                self.parking_sequence = 4

        elif self.parking_sequence == 4:
            ranges = []
            for i in range(150, 210):
                if self.scan.ranges[i] != 0:
                    ranges.append(self.scan.ranges[i])
            if min(ranges) > 0.2:
                cmd_vel.linear.x = -0.04
                cmd_vel.angular.z = 0.0
            else:
                self.get_logger().info("Auto parking Done.")
                self._stop_and_reset()
                sys.exit()

        self.cmd_vel_publisher.publish(cmd_vel)


def main(args=None):
    rclpy.init(args=args)
    automatic_parking = AutomaticParking()

    rclpy.spin(automatic_parking)

    automatic_parking.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
