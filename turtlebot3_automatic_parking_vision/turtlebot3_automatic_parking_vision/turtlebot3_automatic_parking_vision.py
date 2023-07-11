#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################################################################
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
################################################################################

# Authors: Leon Jung, Gilbert

from enum import Enum
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ros2_aruco_interfaces.msg import ArucoMarkers

import numpy as np

MARKER_ID_DETECTION = 17


class AutomaticParkingVision(Node):

    def __init__(self):
        super().__init__('automatic_parking_vision')

        self.sub_odom_robot = self.create_subscription(
            Odometry,
            '/odom',
            self._get_odom,
            qos_profile=QoSProfile(depth=10))

        self.sub_info_marker = self.create_subscription(
            ArucoMarkers,
            '/aruco_markers',
            self._get_aruco_markers,
            qos_profile=QoSProfile(depth=10))

        self.pub_cmd_vel = self.create_publisher(
            Twist,
            '/cmd_vel',
            qos_profile=QoSProfile(depth=10))

        self.is_marker_received = False
        self.timer = self.create_timer(0.05, self._timer_callback)

    def _timer_callback(self):
        self.position_error.x = self.goal_position.x - self.position.x
        self.position_error.y = self.goal_position.y - self.position.y

        distance = math.sqrt(pow(self.position_error.x, 2) + pow(self.position_error.y, 2))
        goal_direction = math.atan2(self.position_error.y, self.position_error.x)

        if distance > 0.1:
            path_angle = goal_direction - self.heading

            if path_angle < -math.pi:
                path_angle = path_angle + 2 * math.pi
            elif path_angle > math.pi:
                path_angle = path_angle - 2 * math.pi

            self.cmd_vel.angular.z = path_angle
            self.cmd_vel.linear.x = min(self.linear_speed * distance, 0.1)

            if self.cmd_vel.angular.z > 0:
                self.cmd_vel.angular.z = min(self.cmd_vel.angular.z, 1.5)
            else:
                self.cmd_vel.angular.z = max(self.cmd_vel.angular.z,  -1.5)
            self.cmd_vel_pub.publish(self.cmd_vel)

        else:
            self.heading_error = self.goal_heading - self.heading

            if self.heading_error < -math.pi:
                self.heading_error = self.heading_error+ 2 * math.pi
            elif self.heading_error > math.pi:
                self.heading_error = self.heading_error- 2 * math.pi

            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = self.heading_error

            if abs(self.heading_error * 180.0 / math.pi) < 1.0:
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = 0.0

        self.get_logger().info("distance: " + str(distance))
        self.get_logger().info("heading_angle: " + str(self.heading_error * 180.0 / math.pi))
        self.cmd_vel_pub.publish(self.cmd_vel)

    def _get_odom(self, msg):
        self.position = msg.pose.pose.position
        _, _, self.heading = self.euler_from_quaternion(msg.pose.pose.orientation)
        # self.get_logger().info('heading: ' + str(self.heading))

    def _get_aruco_markers(self, msg):
        if not self.is_marker_received:
            for i in range(len(msg.marker_ids)):
                if msg.marker_ids[i] == MARKER_ID_DETECTION:
                    if self.is_marker_pose_received == False:
                        self.is_marker_pose_received = True

                    pos_x, pos_y, theta = self.fnGet2DMarkerPose(msg.poses[i])
                    self._set_goal_position(pos_x, pos_y, theta - math.pi)
                    self.get_logger().info('marker detected {} {} {}'.format(pos_x, pos_y, theta))
                    self.is_marker_received = True

    def _set_goal_position(self, x, y, theta):
        self.goal_position.x = x
        self.goal_position.y = y

        self.goal_heading = theta
        if self.goal_heading >= math.pi:
            self.goal_heading = self.goal_heading % (math.pi * 180.0 / math.pi)
        elif self.goal_heading <= -math.pi:
            self.goal_heading = -(-self.goal_heading % (math.pi * 180.0 / math.pi))

        self.goal_heading = self.goal_heading * math.pi / 180.0

        self.get_logger().info(str(self.goal_position.x) + str(self.goal_position.y) + str(self.goal_heading))

    def euler_from_quaternion(self, quat):
        """
        Convert quaternion (w in last place) to euler roll, pitch, yaw.

        quat = [x, y, z, w]
        """
        x = quat.x
        y = quat.y
        z = quat.z
        w = quat.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw


def main(args=None):
    rclpy.init(args=args)
    node = AutomaticParkingVision()

    rclpy.spin(node)

    node.fnShutDown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
