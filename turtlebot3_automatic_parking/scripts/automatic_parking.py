#!/usr/bin/env python
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

import rospy
import sys
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
import numpy as np
from math import sin, cos, pi, atan2
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import time

class ReturnValue(object):
    def __init__(self, name):
        self.name = name

    def retun_val(self, stats, data_1, data_2, data_3):
        self.stats = stats
        self.data_1 = data_1
        self.data_2 = data_2
        self.data_3 = data_3

def scan_parking_spot():
    stats = False
    intensity_index = []
    index_count = []
    spot_angle_index = []
    minimun_scan_angle = 30
    maximun_scan_angle = 330
    intensity_threshold = 100

    for i in range(360):
        if i >= minimun_scan_angle and i < maximun_scan_angle:
            spot_intensity = msg.intensities[i] ** 2 * msg.ranges[i] / 100000
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
                stats = True
            else:
                stats = False
    center_angle = spot_angle_index[int(len(spot_angle_index) / 2)]
    start_angle = spot_angle_index[2]
    end_angle = spot_angle_index[-3]
    spot_angle.retun_val(stats, center_angle, start_angle, end_angle)

def quaternion():
    quaternion = (
        odom.pose.pose.orientation.x,
        odom.pose.pose.orientation.y,
        odom.pose.pose.orientation.z,
        odom.pose.pose.orientation.w)
    euler = euler_from_quaternion(quaternion)
    yaw = euler[2]
    return yaw

def get_angle_distance(angle):
    distance = msg.ranges[int(angle)]
    if msg.ranges[int(angle)] is not None and distance is not 0:
        angle = int(angle)
        distance = distance
    return angle, distance

def get_point(start_angle_distance):
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
    return x, y

def finding_spot_position():
    print("scan parking spot done!")
    stats = False
    start_angle_distance = get_angle_distance(spot_angle.data_1)
    center_angle_distance = get_angle_distance(spot_angle.data_2)
    end_angle_distance = get_angle_distance(spot_angle.data_3)

    if start_angle_distance[1] != 0 and center_angle_distance[1] != 0 and end_angle_distance[1] != 0:
        print("calibration......")
        start_point = get_point(start_angle_distance)
        center_point = get_point(center_angle_distance)
        end_point = get_point(end_angle_distance)
        stats = True
    else:
        stats = False
        print("wrong scan!!")

    return spot_point.retun_val(stats, start_point, center_point, end_point)

def rotate_origin_only(x, y, radians):
    xx = x * cos(radians) + y * sin(radians)
    yy = -x * sin(radians) + y * cos(radians)
    return xx, yy

def scan_spot_filter(msg):
    scan_spot_pub = rospy.Publisher("/scan_spot", LaserScan, queue_size=1)
    scan_spot = msg
    scan_spot_list = list(scan_spot.intensities)
    for i in range(360):
        scan_spot_list[i] = 0
    scan_spot_list[spot_angle.data_1] = msg.ranges[spot_angle.data_1] + 10000
    scan_spot_list[spot_angle.data_2] = msg.ranges[spot_angle.data_2] + 10000
    scan_spot_list[spot_angle.data_3] = msg.ranges[spot_angle.data_3] + 10000
    scan_spot.intensities = tuple(scan_spot_list)
    scan_spot_pub.publish(scan_spot)

if __name__=="__main__":
    rospy.init_node('AutoParking')
    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    reset_pub = rospy.Publisher('/reset', Empty, queue_size=1)
    msg = LaserScan()
    r = rospy.Rate(10)
    spot_angle = ReturnValue('spot_angle')
    spot_point = ReturnValue('spot_point')
    step = 0
    twist = Twist()
    reset = Empty()
    while not rospy.is_shutdown():
        msg = rospy.wait_for_message("/scan", LaserScan)
        odom = rospy.wait_for_message("/odom", Odometry)
        yaw = quaternion()
        scan_parking_spot()
        if step == 0:
            if spot_angle.stats == True:
                finding_spot_position()
                if spot_point.stats == True:
                    theta = np.arctan2(spot_point.data_1[1] - spot_point.data_3[1], spot_point.data_1[0] - spot_point.data_3[0])
                    print("=================================")
                    print("|        |     x     |     y     |")
                    print('| start  | {0:>10.3f}| {1:>10.3f}|'.format(spot_point.data_1[0], spot_point.data_1[1]))
                    print('| center | {0:>10.3f}| {1:>10.3f}|'.format(spot_point.data_2[0], spot_point.data_2[1]))
                    print('| end    | {0:>10.3f}| {1:>10.3f}|'.format(spot_point.data_3[0], spot_point.data_3[1]))
                    print("=================================")
                    print('| theta  | {0:.2f} deg'.format(np.rad2deg(theta)))
                    print('| yaw    | {0:.2f} deg'.format(np.rad2deg(yaw)))
                    print("=================================")
                    print("===== Go to parking spot!!! =====")
                    step = 1
            else:
                print("Fail finding parking spot.")
        elif step == 1:
            init_yaw = yaw
            yaw = theta + yaw
            if theta > 0:
                if theta - init_yaw > 0.1:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.2
                else:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    cmd_pub.publish(twist)
                    time.sleep(1)
                    reset_pub.publish(reset)
                    time.sleep(3)
                    rotation_point = rotate_origin_only(spot_point.data_2[0], spot_point.data_2[1], -(pi / 2 - init_yaw))
                    step = 2
            else:
                if theta - init_yaw < -0.1:
                    twist.linear.x = 0.0
                    twist.angular.z = -0.2
                else:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    cmd_pub.publish(twist)
                    time.sleep(1)
                    reset_pub.publish(reset)
                    time.sleep(3)
                    rotation_point = rotate_origin_only(spot_point.data_2[0], spot_point.data_2[1], -(pi / 2 - init_yaw))
                    step = 2
        elif step == 2:
            if abs(odom.pose.pose.position.x - (rotation_point[1])) > 0.02:
                if odom.pose.pose.position.x > (rotation_point[1]):
                    twist.linear.x = -0.05
                    twist.angular.z = 0.0
                else:
                    twist.linear.x = 0.05
                    twist.angular.z = 0.0
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                step = 3
        elif step == 3:
            if yaw > -pi / 2:
                twist.linear.x = 0.0
                twist.angular.z = -0.2
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                step = 4
        elif step == 4:
            ranges = []
            for i in range(150, 210):
                if msg.ranges[i] != 0:
                    ranges.append(msg.ranges[i])
            if min(ranges) > 0.2:
                twist.linear.x = -0.04
                twist.angular.z = 0.0
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                print("Auto_parking Done.")
                cmd_pub.publish(twist)
                sys.exit()
        cmd_pub.publish(twist)
        scan_spot_filter(msg)
