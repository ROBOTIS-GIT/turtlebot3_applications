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
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from ros2_aruco_interfaces.msg import ArucoMarkers

from tf_transformations import euler_from_quaternion
import numpy as np

MARKER_ID_DETECTION = 17


class AutomaticParkingVision(Node):

    def __init__(self):
        super().__init__('automatic_parking_vision')

        self.is_marker_received = False
        self.is_set_goal = False
        self.goal_position = Point()
        self.goal_heading = 0.0
        self.heading = 0.0
        self.position = Point()
        self.position_error = Point()
        self.heading_error = 0.0
        self.angular_speed = 0.3
        self.linear_speed = 0.5
        self.queue_pose_x = []
        self.queue_pose_y = []
        self.queue_pose_theta = []
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

        self.timer = self.create_timer(0.05, self._timer_callback)

    def _timer_callback(self):
        if self.is_set_goal:
            self.get_logger().info('self.position {}'.format(self.position))
            self.position_error.x = self.goal_position.x - self.position.x
            self.position_error.y = self.goal_position.y - self.position.y

            distance = math.sqrt(pow(self.position_error.x, 2) + pow(self.position_error.y, 2))
            goal_direction = math.atan2(self.position_error.y, self.position_error.x)
            cmd_vel = Twist()
            if distance > 0.05:
                path_angle = goal_direction - self.heading

                if path_angle < -math.pi:
                    path_angle = path_angle + 2 * math.pi
                elif path_angle > math.pi:
                    path_angle = path_angle - 2 * math.pi

                cmd_vel.angular.z = path_angle
                cmd_vel.linear.x = min(self.linear_speed * distance, 0.1)

                if cmd_vel.angular.z > 0:
                    cmd_vel.angular.z = min(cmd_vel.angular.z, 1.5)
                else:
                    cmd_vel.angular.z = max(cmd_vel.angular.z,  -1.5)

            else:
                self.heading_error = self.goal_heading - self.heading

                if self.heading_error < -math.pi:
                    self.heading_error = self.heading_error+ 2 * math.pi
                elif self.heading_error > math.pi:
                    self.heading_error = self.heading_error- 2 * math.pi

                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = self.heading_error

                if abs(self.heading_error * 180.0 / math.pi) < 0.2:
                    cmd_vel.linear.x = 0.0
                    cmd_vel.angular.z = 0.0

            self.get_logger().info("distance: " + str(distance))
            self.get_logger().info("heading_angle: " + str(self.goal_heading * 180 / math.pi))
            self.get_logger().info("goal_heading: " + str(self.heading * 180 / math.pi))
            self.pub_cmd_vel.publish(cmd_vel)

    def _get_odom(self, msg):
        self.position = msg.pose.pose.position
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        self.heading = euler_from_quaternion(quaternion)[2]
        # self.get_logger().info('heading: ' + str(self.heading))

    def _median_filter(self, data):
        filtered_data = []
        window_size = 5

        for i in range(len(data)):
            start_index = max(0, i - window_size + 1)
            window = data[start_index:i+1]
            median = sorted(window)[len(window) // 2]
            filtered_data.append(median)

        return filtered_data[-1]

    def _get_aruco_markers(self, msg):
        if not self.is_marker_received:
            for i in range(len(msg.marker_ids)):
                if msg.marker_ids[i] == MARKER_ID_DETECTION:
                    pos_x, pos_y, theta = self._get_marker_pose(msg.poses[i])
                    self.queue_pose_x.append(pos_x)
                    self.queue_pose_y.append(pos_y)
                    self.queue_pose_theta.append(theta - math.pi)
                    if len(self.queue_pose_x) > 10:
                        median_pose_x = self._median_filter(self.queue_pose_x)
                        median_pose_y = self._median_filter(self.queue_pose_y)
                        median_pose_theta = self._median_filter(self.queue_pose_theta)
                        self.get_logger().info("marker received: {} {} {}".format(median_pose_x, median_pose_y, median_pose_theta))
                        self._set_goal_position(median_pose_x, median_pose_y, median_pose_theta)
                        self.queue_pose_x = []
                        self.queue_pose_y = []
                        self.queue_pose_theta = []
                        self.is_marker_received = True

    def _rotate_pose(self, pose):
        self.get_logger().info("pose {0}".format(pose.position))
        rotation_x = math.pi / 2
        cos_angle_x = math.cos(rotation_x)
        sin_angle_x = math.sin(rotation_x)
        rotation_matrix_x = [[1, 0, 0],
                            [0, cos_angle_x, -sin_angle_x],
                            [0, sin_angle_x, cos_angle_x]]

        rotation_z = math.pi / 2
        cos_angle_z = math.cos(rotation_z)
        sin_angle_z = math.sin(rotation_z)
        rotation_matrix_z = [[cos_angle_z, -sin_angle_z, 0],
                            [sin_angle_z, cos_angle_z, 0],
                            [0, 0, 1]]

        pose_matrix = [[pose.position.x],
                    [pose.position.y],
                    [pose.position.z]]
        rotated_pose_matrix = np.dot(rotation_matrix_z, np.dot(rotation_matrix_x, pose_matrix))

        orientation = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        rotated_orientation = [0, 0, 0, 0]
        rotated_orientation[0] = cos_angle_x * cos_angle_z * orientation[0] - sin_angle_x * sin_angle_z * orientation[1]
        rotated_orientation[1] = sin_angle_x * cos_angle_z * orientation[0] + cos_angle_x * sin_angle_z * orientation[1]
        rotated_orientation[2] = cos_angle_x * sin_angle_z * orientation[0] + sin_angle_x * cos_angle_z * orientation[2]
        rotated_orientation[3] = cos_angle_x * cos_angle_z * orientation[3] - sin_angle_x * sin_angle_z * orientation[3]

        rotated_pose = Pose()
        rotated_pose.position.x = rotated_pose_matrix[0][0]
        rotated_pose.position.y = -rotated_pose_matrix[1][0]
        rotated_pose.position.z = rotated_pose_matrix[2][0]
        rotated_pose.orientation.x = rotated_orientation[0]
        rotated_pose.orientation.y = rotated_orientation[1]
        rotated_pose.orientation.z = rotated_orientation[2]
        rotated_pose.orientation.w = rotated_orientation[3]

        return rotated_pose

    def _get_marker_pose(self, marker_pose):
            pose = self._rotate_pose(marker_pose)
            self.get_logger().info("rotation pose {0}".format(pose.position))
            quaternion = (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w)
            theta = euler_from_quaternion(quaternion)[2]
            # theta = theta + np.pi / 2.

            # if theta < 0.0:
            #     theta = theta + np.pi * 2
            # if theta > np.pi * 2:
            #     theta = theta - np.pi * 2

            pos_x = pose.position.x
            pos_y = pose.position.y

            return pos_x, pos_y, theta

    def _set_goal_position(self, x, y, theta):
        self.goal_position.x = x
        self.goal_position.y = y

        self.goal_heading = math.degrees(theta)
        if self.goal_heading >= math.pi:
            self.goal_heading = self.goal_heading % (math.pi * 180.0 / math.pi)
        elif self.goal_heading <= -math.pi:
            self.goal_heading = -(-self.goal_heading % (math.pi * 180.0 / math.pi))

        self.goal_heading = self.goal_heading * math.pi / 180.0
        self.is_set_goal = True
        self.get_logger().info(str(self.goal_position.x) + str(self.goal_position.y) + str(self.goal_heading))

    def _euler_from_quaternion(self, quat):
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
