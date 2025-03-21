#!/usr/bin/env python3
#################################################################################
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
#################################################################################

# Authors: Gilbert, YeonSoo Noh

from math import sin, cos, pi
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty

import numpy as np
from transforms3d.euler import quat2euler


class AutomaticParking(Node):

    def __init__(self):
        super().__init__('automatic_parking')

        # Set initial value
        self.scan = None
        self.odom = None
        self.search_count = 0
        self.euler = [0.0, 0.0, 0.0]
        self.center_index = None
        self.start_index = None
        self.end_index = None
        self.start_point = [0.0, 0.0]
        self.end_point = [0.0, 0.0]
        self.center_point = [0.0, 0.0]
        self.new_center = [0.0, 0.0]
        self.parking_sequence = 0
        self.theta = 0.0
        self.is_scan_received = False
        self.is_odom_received = False
        self.init_yaw = None
        self.init_x = None
        self.target_yaw = None

        self.prev_yaw = None
        self.total_yaw = 0.0 
                
        # Set publisher
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            qos_profile=QoSProfile(depth=100))

        self.reset_publisher = self.create_publisher(
            Empty,
            '/reset',
            qos_profile=QoSProfile(depth=100))

        self.scan_spot_publisher = self.create_publisher(
            LaserScan,
            '/scan_spot',
            qos_profile=qos_profile_sensor_data)

        # Set subscriber
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self._scan_callback,
            qos_profile=qos_profile_sensor_data)

        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self._odom_callback,
            qos_profile=QoSProfile(depth=100))

        self._run_timer = self.create_timer(0.05, self._run)

    def _scan_callback(self, msg):
        self.scan = msg
        scan_spot = LaserScan()
        scan_spot.header = msg.header
        scan_spot.angle_min = msg.angle_min
        scan_spot.angle_max = msg.angle_max
        scan_spot.angle_increment = msg.angle_increment
        scan_spot.time_increment = msg.time_increment
        scan_spot.scan_time = msg.scan_time
        scan_spot.range_min = msg.range_min
        scan_spot.range_max = msg.range_max
        scan_spot.ranges = msg.ranges
        if self.start_index != None and self.center_index != None and self.end_index != None:
            for i in range(len(msg.intensities)):
                if i == self.start_index or i == self.center_index or i == self.end_index:
                    scan_spot.intensities.append(msg.intensities[i] + 10000)
                else:
                    scan_spot.intensities.append(msg.intensities[i])
        self.scan_spot_publisher.publish(scan_spot)
        self.is_scan_received = True

    def _odom_callback(self, msg):
        self.odom = msg
        self.is_odom_received = True

    def _get_point(self, angle_distance):
        angle = angle_distance[0]
        distance = angle_distance[1]

        x = distance * cos(angle) 
        y = distance * sin(angle)

        return [x, y]

    def _get_angle_distance(self, index):
        distance = self.scan.ranges[index]
        angle = self.scan.angle_min + (index * self.scan.angle_increment)
        return angle, distance

    def _scan_parking_spot(self):
        scan_done = False
        intensity_index = []
        index_count = []
        spot_angle_index = []
        intensity_threshold = 200 

        if self.scan != None:
            for i in range(len(self.scan.ranges)):
                spot_intensity = self.scan.intensities[i]
                if not math.isnan(spot_intensity) and spot_intensity >= intensity_threshold:
                    intensity_index.append(i)
                    index_count.append(i)
                else:
                    intensity_index.append(0)
            for i in index_count:
                if abs(i - index_count[int(len(index_count) / 2)]) < 20:
                    spot_angle_index.append(i)
                    if len(spot_angle_index) > 10: 
                        scan_done = True
                        self.center_index = spot_angle_index[int(len(spot_angle_index) / 2)]
                        self.start_index = spot_angle_index[2]
                        self.end_index = spot_angle_index[-3]

        return scan_done

    def _finding_spot_position(self):
        self.get_logger().info("scan parking spot done!")
        center_angle_distance = self._get_angle_distance(self.center_index)
        start_angle_distance = self._get_angle_distance(self.start_index)
        end_angle_distance = self._get_angle_distance(self.end_index)

        if start_angle_distance[1] != 0 and center_angle_distance[1] != 0 and end_angle_distance[1] != 0:
            self.get_logger().info("calibration......")
            self.center_point = self._get_point(center_angle_distance)
            self.start_point = self._get_point(start_angle_distance)
            self.end_point = self._get_point(end_angle_distance)

            theta1 = np.arctan2(
                self.start_point[1] - self.end_point[1],
                self.start_point[0] - self.end_point[0])
            theta2 = np.arctan2(
                self.end_point[1] - self.start_point[1],
                self.end_point[0] - self.start_point[0])
            
            self.theta = theta1 if abs(theta1) < abs(theta2) else theta2
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
        self.new_center[0] = self.center_point[0] * cos(radians) + self.center_point[1] * sin(radians)
        self.new_center[1] = -self.center_point[0] * sin(radians) + self.center_point[1] * cos(radians)

    def _stop_and_reset(self):
        cmd_vel = Twist()
        reset = Empty()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_publisher.publish(cmd_vel)
        time.sleep(1)
        self.reset_publisher.publish(reset)
        time.sleep(3)

    def _get_yaw(self):
        orientation = self.odom.pose.pose.orientation 
        quaternion = (
            orientation.w,
            orientation.x,
            orientation.y,
            orientation.z
        )
        euler = quat2euler(quaternion)
        yaw = euler[2]

        if self.prev_yaw is None:
            self.prev_yaw = yaw

        delta_yaw = yaw - self.prev_yaw

        if delta_yaw > math.pi:
            self.total_yaw -= 2 * math.pi
        elif delta_yaw < -math.pi:
            self.total_yaw += 2 * math.pi 

        self.total_yaw += delta_yaw 
        self.prev_yaw = yaw 

        return self.total_yaw
    

    def _run(self):
        if self.is_scan_received and self.is_odom_received:
            yaw = self._get_yaw()
            if self.init_yaw is None:
                    self.init_yaw = yaw
            cmd_vel = Twist()
            ranges = []

            if self.parking_sequence == 0:
                self.get_logger().info("Start auto parking!")
                self.parking_sequence += 1
            elif self.parking_sequence == 1:
                if self._scan_parking_spot():
                    if self._finding_spot_position():
                        self._print_parking_log()
                        self.parking_sequence += 1
                        self.get_logger().info("Rotation!")
                else:
                    self.search_count += 1
                    if self.search_count > 100:
                        self.get_logger().error("Fail to finding parking spot.")
                        self.search_count = 0

            elif self.parking_sequence == 2:
                relative_yaw = yaw - self.init_yaw
                if self.theta > 0:
                    if self.theta - relative_yaw > 0.1:
                        cmd_vel.linear.x = 0.0
                        cmd_vel.angular.z = 0.2
                    else:
                        self._stop_and_reset()
                        self._rotate_origin_only(relative_yaw)
                        self.parking_sequence += 1
                        self.get_logger().info("Go to parking spot!")
                else:
                    if self.theta - relative_yaw < -0.1:
                        cmd_vel.linear.x = 0.0
                        cmd_vel.angular.z = -0.2
                    else:
                        self._stop_and_reset()
                        self._rotate_origin_only(relative_yaw)
                        self.parking_sequence += 1
                        self.get_logger().info("Go to parking spot!")

            elif self.parking_sequence == 3:
                if self.init_x is None:
                    self.init_x = self.odom.pose.pose.position.x
                current_x = self.odom.pose.pose.position.x
                moved_distance = abs(current_x - self.init_x)
                if abs(self.new_center[0]) - moved_distance > 0:
                    if self.new_center[0] < 0.02:
                        cmd_vel.linear.x = -0.05
                        cmd_vel.angular.z = 0.0
                    else:
                        cmd_vel.linear.x = 0.05
                        cmd_vel.angular.z = 0.0
                else:
                    cmd_vel.linear.x = 0.0
                    cmd_vel.angular.z = 0.0
                    self.parking_sequence += 1
                    self.get_logger().info("Rotation Done.")

            elif self.parking_sequence == 4:
                if self.new_center[1] > 0: 
                    if self.target_yaw is None:
                        self.target_yaw = yaw - (pi / 2)
                    if yaw - self.target_yaw > 0.1:
                        cmd_vel.linear.x = 0.0
                        cmd_vel.angular.z = -0.2
                    else:
                        cmd_vel.linear.x = 0.0
                        cmd_vel.angular.z = 0.0
                        self.parking_sequence += 1
                        self.get_logger().info("Approach spot.")
                    
                else:  
                    if self.target_yaw is None:
                        self.target_yaw = yaw + (pi / 2)
                    if self.target_yaw - yaw > 0.1:
                        cmd_vel.linear.x = 0.0
                        cmd_vel.angular.z = 0.2
                    else:
                        cmd_vel.linear.x = 0.0
                        cmd_vel.angular.z = 0.0
                        self.parking_sequence += 1
                        self.get_logger().info("Approach spot.")


            elif self.parking_sequence == 5:
                ranges = []
                min_scan_index = int((math.radians(150) - self.scan.angle_min) / self.scan.angle_increment)
                max_scan_index = int((math.radians(210) - self.scan.angle_min) / self.scan.angle_increment)
                for i in range(min_scan_index, max_scan_index):
                    if self.scan.ranges[i] != 0:
                        ranges.append(self.scan.ranges[i])
                valid_ranges = [r for r in ranges if not math.isnan(r)]
                if valid_ranges:  
                    min_distance = min(valid_ranges)
                else:
                    min_distance = 0  

                if min_distance > 0.13:
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

    try:
        rclpy.spin(automatic_parking)
    except KeyboardInterrupt:
        automatic_parking.get_logger().info('Stopping robot...')
    finally:
        automatic_parking._stop_and_reset()
        automatic_parking.destroy_node
        rclpy.shutdown()


if __name__ == '__main__':
    main()