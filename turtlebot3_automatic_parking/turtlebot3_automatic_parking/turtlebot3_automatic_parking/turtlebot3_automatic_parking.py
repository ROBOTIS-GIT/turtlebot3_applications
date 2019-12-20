#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
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
# Authors: Gilbert, Ryan Shim

import math
import numpy
import sys
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty


class Turtlebot3AutomaticParking(Node):
    def __init__(self):
        super().__init__('turtlebot3_automatic_parking')

        """************************************************************
        ** Initialise variables
        ************************************************************"""
        self.odom = Odometry()
        self.last_pose_x = 0.0
        self.last_pose_y = 0.0
        self.last_pose_theta = 0.0
        self.goal_pose_x = 0.0
        self.goal_pose_y = 0.0
        self.goal_pose_theta = 0.0
        self.step = 0
        self.scan = []
        self.rotation_point = []
        self.center_point = []
        self.theta = 0.0
        self.yaw = 0.0
        self.get_key_state = False
        self.init_scan_state = False  # To get the initial scan at the beginning
        self.init_odom_state = False  # To get the initial odom at the beginning

        """************************************************************
        ** Initialise ROS publishers and subscribers
        ************************************************************"""
        qos = QoSProfile(depth=10)

        # Initialise publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)
        self.reset_pub = self.create_publisher(Empty, 'reset', qos)
        self.scan_spot_pub = self.create_publisher(LaserScan, 'scan_spot', qos)

        # Initialise subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile=qos_profile_sensor_data)
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            qos)

        """************************************************************
        ** Initialise timers
        ************************************************************"""
        self.update_timer = self.create_timer(
            0.010,  # unit: s
            self.update_callback)

        self.get_logger().info("Turtlebot3 automatic parking node has been initialised.")

    """*******************************************************************************
    ** Callback functions and relevant functions
    *******************************************************************************"""
    def scan_callback(self, msg):
        self.scan = msg
        self.init_scan_state = True

    def odom_callback(self, msg):
        self.odom = msg
        self.last_pose_x = msg.pose.pose.position.x
        self.last_pose_y = msg.pose.pose.position.y
        _, _, self.last_pose_theta = self.euler_from_quaternion(msg.pose.pose.orientation)
        self.yaw = self.last_pose_theta
        self.init_odom_state = True

    def update_callback(self):
        if self.init_scan_state is True and self.init_odom_state is True:
            self.park_robot()

    def park_robot(self):
        scan_done, center_angle, start_angle, end_angle = self.scan_parking_spot()
        twist = Twist()
        reset = Empty()

        # Step 0: Find a parking spot
        if self.step == 0:
            if scan_done == True:
                fining_spot, start_point, self.center_point, end_point = self.find_parking_spot(center_angle, start_angle, end_angle)
                if fining_spot == True:
                    self.theta = numpy.arctan2(start_point[1] - end_point[1], start_point[0] - end_point[0])
                    print("=================================")
                    print("|        |     x     |     y     |")
                    print('| start  | {0:>10.3f}| {1:>10.3f}|'.format(start_point[0], start_point[1]))
                    print('| center | {0:>10.3f}| {1:>10.3f}|'.format(self.center_point[0], self.center_point[1]))
                    print('| end    | {0:>10.3f}| {1:>10.3f}|'.format(end_point[0], end_point[1]))
                    print("=================================")
                    print('| theta  | {0:.2f} deg'.format(numpy.rad2deg(self.theta)))
                    print('| yaw    | {0:.2f} deg'.format(numpy.rad2deg(self.yaw)))
                    print("=================================")
                    print("===== Go to parking spot!!! =====")
                    self.step = 1
            else:
                print("Fail finding parking spot.")

        # Step 1: Turn
        elif self.step == 1:
            init_yaw = self.yaw
            self.yaw = self.theta + self.yaw
            if self.theta > 0:
                if self.theta - init_yaw > 0.1:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.2
                else:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.cmd_vel_pub.publish(twist)
                    time.sleep(1)
                    self.reset_pub.publish(reset)
                    time.sleep(3)
                    self.rotation_point = self.rotate_origin_only(self.center_point[0], self.center_point[1], -(math.pi / 2 - init_yaw))
                    self.step = 2
            else:
                if self.theta - init_yaw < -0.1:
                    twist.linear.x = 0.0
                    twist.angular.z = -0.2
                else:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.cmd_vel_pub.publish(twist)
                    time.sleep(1)
                    self.reset_pub.publish(reset)
                    time.sleep(3)
                    self.rotation_point = self.rotate_origin_only(self.center_point[0], self.center_point[1], -(math.pi / 2 - init_yaw))
                    self.step = 2

        # Step 2: Move straight
        elif self.step == 2:
            if abs(self.odom.pose.pose.position.x - (self.rotation_point[1])) > 0.02:
                if self.odom.pose.pose.position.x > (self.rotation_point[1]):
                    twist.linear.x = -0.05
                    twist.angular.z = 0.0
                else:
                    twist.linear.x = 0.05
                    twist.angular.z = 0.0
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.step = 3

        # Step 3: Turn
        elif self.step == 3:
            if self.yaw > -math.pi / 2:
                twist.linear.x = 0.0
                twist.angular.z = -0.2
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.step = 4

        # Step 4: Move Straight
        elif self.step == 4:
            ranges = []
            for i in range(150, 210):
                if self.scan.ranges[i] != 0:
                    ranges.append(self.scan.ranges[i])
            if min(ranges) > 0.2:
                twist.linear.x = -0.04
                twist.angular.z = 0.0
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                print("Automatic parking done.")
                self.cmd_vel_pub.publish(twist)
                sys.exit()
                
        self.cmd_vel_pub.publish(twist)
        self.scan_spot_filter(center_angle, start_angle, end_angle)

    def scan_parking_spot(self):
        scan_done = False
        intensity_index = []
        index_count = []
        spot_angle_index = []
        minimun_scan_angle = 30
        maximun_scan_angle = 330
        intensity_threshold = 100
        center_angle = 0
        start_angle = 0
        end_angle = 0
        for i in range(360):
            if i >= minimun_scan_angle and i < maximun_scan_angle:
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
                    center_angle = spot_angle_index[int(len(spot_angle_index) / 2)]
                    start_angle = spot_angle_index[2]
                    end_angle = spot_angle_index[-3]

                else:
                    scan_done = False
        return scan_done, center_angle, start_angle, end_angle

    def get_angle_distance(self, angle):
        distance = self.scan.ranges[int(angle)]
        if self.scan.ranges[int(angle)] is not None and distance is not 0:
            angle = int(angle)
            distance = distance
        return angle, distance

    def get_point(self, start_angle_distance):
        angle = start_angle_distance[0]
        angle = numpy.deg2rad(angle - 180)
        distance = start_angle_distance[1]

        if angle >= 0 and angle < math.pi / 2:
            x = distance * math.cos(angle) * -1
            y = distance * math.sin(angle) * -1
        elif angle >= math.pi / 2 and angle < math.pi:
            x = distance * math.cos(angle) * -1
            y = distance * math.sin(angle) * -1
        elif angle >= -math.pi / 2 and angle < 0:
            x = distance * math.cos(angle) * -1
            y = distance * math.sin(angle) * -1
        else:
            x = distance * math.cos(angle) * -1
            y = distance * math.sin(angle) * -1

        return [x, y]

    def find_parking_spot(self, center_angle, start_angle, end_angle):
        print("scan parking spot done!")
        fining_spot = False
        start_angle_distance = self.get_angle_distance(start_angle)
        center_angle_distance = self.get_angle_distance(center_angle)
        end_angle_distance = self.get_angle_distance(end_angle)

        if start_angle_distance[1] != 0 and center_angle_distance[1] != 0 and end_angle_distance[1] != 0:
            print("calibration......")
            start_point = self.get_point(start_angle_distance)
            center_point = self.get_point(center_angle_distance)
            end_point = self.get_point(end_angle_distance)
            fining_spot = True
        else:
            fining_spot = False
            print("wrong scan!!")

        return fining_spot, start_point, center_point, end_point

    def rotate_origin_only(self, x, y, radians):
        xx = x * math.cos(radians) + y * math.sin(radians)
        yy = -x * math.sin(radians) + y * math.cos(radians)
        return xx, yy

    def scan_spot_filter(self, center_angle, start_angle, end_angle):
        scan_spot = self.scan
        scan_spot_list = list(scan_spot.intensities)
        for i in range(360):
            scan_spot_list[i] = 0.0
        scan_spot_list[start_angle] = self.scan.ranges[start_angle] + 10000
        scan_spot_list[center_angle] = self.scan.ranges[center_angle] + 10000
        scan_spot_list[end_angle] = self.scan.ranges[end_angle] + 10000
        scan_spot.intensities = tuple(scan_spot_list)
        self.scan_spot_pub.publish(scan_spot)

    """*******************************************************************************
    ** Below should be replaced when porting for ROS 2 Python tf_conversions is done.
    *******************************************************************************"""
    def euler_from_quaternion(self, quat):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quat = [x, y, z, w]
        """
        x = quat.x
        y = quat.y
        z = quat.z
        w = quat.w

        sinr_cosp = 2 * (w*x + y*z)
        cosr_cosp = 1 - 2*(x*x + y*y)
        roll = numpy.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w*y - z*x)
        pitch = numpy.arcsin(sinp)

        siny_cosp = 2 * (w*z + x*y)
        cosy_cosp = 1 - 2 * (y*y + z*z)
        yaw = numpy.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw
