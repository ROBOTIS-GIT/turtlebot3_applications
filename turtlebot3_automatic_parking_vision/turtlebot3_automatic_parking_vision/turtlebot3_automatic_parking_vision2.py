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
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from ros2_aruco_interfaces.msg import ArucoMarkers
from tf2_geometry_msgs import do_transform_pose

from tf_transformations import euler_from_quaternion
import numpy as np


MARKER_ID_DETECTION = 17


class AutomaticParkingVision(Node):

    def __init__(self):
        super().__init__('automatic_parking_vision')

        self.sub_odom_robot = self.create_subscription(
            Odometry,
            '/odom',
            self.cbGetRobotOdom,
            qos_profile=qos_profile_sensor_data)

        self.sub_info_marker = self.create_subscription(
            ArucoMarkers,
            '/aruco_markers',
            self.cbGetMarkerOdom,
            qos_profile=qos_profile_sensor_data)

        self.pub_cmd_vel = self.create_publisher(
            Twist,
            '/cmd_vel',
            qos_profile=QoSProfile(depth=10))


        self.ParkingSequence = Enum(
            'ParkingSequence',
            'searching_parking_lot changing_direction moving_nearby_parking_lot parking stop finished')

        self.NearbySequence = Enum(
            'NearbySequence',
            'initial_turn go_straight turn_right parking')

        self.current_nearby_sequence = self.NearbySequence.initial_turn.value
        self.current_parking_sequence = self.ParkingSequence.searching_parking_lot.value

        self.robot_2d_pose_x = .0
        self.robot_2d_pose_y = .0
        self.robot_2d_theta = .0
        self.marker_2d_pose_x = .0
        self.marker_2d_pose_y = .0
        self.marker_2d_theta = .0

        self.previous_robot_2d_theta = .0
        self.total_robot_2d_theta = .0
        self.is_triggered = False

        self.is_sequence_finished = False

        self.is_odom_received = False
        self.is_marker_pose_received = False

        self.timer = self.create_timer(0.1, self._run)

    def _run(self):
        if self.is_odom_received is True:
            self.fnParking()

    def cbGetRobotOdom(self, robot_odom_msg):
        if self.is_odom_received == False:
            self.is_odom_received = True
            self.get_logger().info('Odometry received')

        pos_x, pos_y, theta = self.fnGet2DRobotPose(robot_odom_msg)

        self.robot_2d_pose_x = pos_x
        self.robot_2d_pose_y = pos_y
        self.robot_2d_theta = theta

        if (self.robot_2d_theta - self.previous_robot_2d_theta) > 5.:
            d_theta = (self.robot_2d_theta - self.previous_robot_2d_theta) - 2 * math.pi
        elif (self.robot_2d_theta - self.previous_robot_2d_theta) < -5.:
            d_theta = (self.robot_2d_theta - self.previous_robot_2d_theta) + 2 * math.pi
        else:
            d_theta = (self.robot_2d_theta - self.previous_robot_2d_theta)

        self.total_robot_2d_theta = self.total_robot_2d_theta + d_theta
        self.previous_robot_2d_theta = self.robot_2d_theta

        self.robot_2d_theta = self.total_robot_2d_theta

    def cbGetMarkerOdom(self, markers_odom_msg):
        for i in range(len(markers_odom_msg.marker_ids)):
            if markers_odom_msg.marker_ids[i] == MARKER_ID_DETECTION:
                if self.is_marker_pose_received == False:
                    self.is_marker_pose_received = True

                pos_x, pos_y, theta = self.fnGet2DMarkerPose(markers_odom_msg.poses[i])

                self.marker_2d_pose_x = pos_x
                self.marker_2d_pose_y = pos_y
                self.marker_2d_theta = theta - math.pi
                self.get_logger().info('Marker pose received: {}, {}, {}'.format(pos_x, pos_y, math.degrees(self.marker_2d_theta)))
    def fnParking(self):
        if self.current_parking_sequence == self.ParkingSequence.searching_parking_lot.value:
            self.is_sequence_finished = self.fnSeqSearchingGoal()

            if self.is_sequence_finished == True:
                self.get_logger().info("Finished 1")
                self.current_parking_sequence = self.ParkingSequence.changing_direction.value
                self.is_sequence_finished = False

        elif self.current_parking_sequence == self.ParkingSequence.changing_direction.value:
            self.get_logger().info("changing_direction")
            self.is_sequence_finished = self.fnSeqChangingDirection()

            if self.is_sequence_finished == True:
                self.get_logger().info("Finished 2")
                self.current_parking_sequence = self.ParkingSequence.moving_nearby_parking_lot.value
                self.is_sequence_finished = False

        elif self.current_parking_sequence == self.ParkingSequence.moving_nearby_parking_lot.value:
            self.get_logger().info("moving_nearby_parking_lot")
            self.is_sequence_finished = self.fnSeqMovingNearbyParkingLot()

            if self.is_sequence_finished == True:
                self.get_logger().info("Finished 3")
                self.current_parking_sequence = self.ParkingSequence.parking.value
                self.is_sequence_finished = False

        elif self.current_parking_sequence == self.ParkingSequence.parking.value:
            self.is_sequence_finished = self.fnSeqParking()

            if self.is_sequence_finished == True:
                self.get_logger().info("Finished 4")
                self.current_parking_sequence = self.ParkingSequence.stop.value
                self.is_sequence_finished = False

        elif self.current_parking_sequence == self.ParkingSequence.stop.value:
            self.fnStop()
            self.get_logger().info("Finished 5")
            self.current_parking_sequence = self.ParkingSequence.finished.value
            self.fnShutDown()
            rclpy.shutdown()

    def fnSeqSearchingGoal(self):
        if self.is_marker_pose_received is False:
            self.desired_angle_turn = -0.6
            self.fnTurn(self.desired_angle_turn)
        else:
            self.fnStop()
            return True

    def fnSeqChangingDirection(self):
        desired_angle_turn = -1. *  math.atan2(self.marker_2d_pose_y - 0, self.marker_2d_pose_x - 0)
        self.get_logger().info(
            "desired_angle_turn {} self.marker_2d_pose_x {} self.marker_2d_pose_y {}".format(
                desired_angle_turn, self.marker_2d_pose_x, self.marker_2d_pose_y))
        # rospy.loginfo("desired_angle_turn %f self.marker_2d_pose_x %f self.marker_2d_pose_y %f"
        # , desired_angle_turn, self.marker_2d_pose_x, self.marker_2d_pose_y)

        self.fnTurn(desired_angle_turn)

        if abs(desired_angle_turn) < 0.1:
            self.fnStop()
            return True
        else:
            return False

    def fnSeqMovingNearbyParkingLot(self):
        if self.current_nearby_sequence == self.NearbySequence.initial_turn.value:
            if self.is_triggered == False:
                self.is_triggered = True
                self.initial_robot_pose_theta = self.robot_2d_theta
                self.initial_robot_pose_x = self.robot_2d_pose_x
                self.initial_robot_pose_y = self.robot_2d_pose_y
                self.initial_marker_pose_theta = self.marker_2d_theta
                self.initial_marker_pose_x = self.marker_2d_pose_x

            if self.initial_marker_pose_theta < 0.0:
                desired_angle_turn = (math.pi / 2.0) + self.initial_marker_pose_theta - (self.robot_2d_theta - self.initial_robot_pose_theta)
            elif self.initial_marker_pose_theta > 0.0:
                desired_angle_turn = -(math.pi / 2.0) + self.initial_marker_pose_theta - (self.robot_2d_theta - self.initial_robot_pose_theta)

            # rospy.loginfo("desired_angle_turn %f self.initial_marker_pose_theta %f self.robot_2d_theta %f self.initial_robot_pose_theta %f"
            # , desired_angle_turn, self.initial_marker_pose_theta, self.robot_2d_theta, self.initial_robot_pose_theta)

            desired_angle_turn = -1. * desired_angle_turn

            self.fnTurn(desired_angle_turn)

            if abs(desired_angle_turn) < 0.05:
                self.fnStop()
                self.current_nearby_sequence = self.NearbySequence.go_straight.value
                self.is_triggered = False

        elif self.current_nearby_sequence == self.NearbySequence.go_straight.value:
            dist_from_start = self.fnCalcDistPoints(self.initial_robot_pose_x, self.robot_2d_pose_x, self.initial_robot_pose_y, self.robot_2d_pose_y)

            desired_dist = self.initial_marker_pose_x * abs(math.cos((math.pi / 2.) - self.initial_marker_pose_theta))
            remained_dist = desired_dist - dist_from_start
            # rospy.loginfo("remained_dist %f desired_dist %f dist_from_start %f", remained_dist, desired_dist, dist_from_start)

            self.fnGoStraight()
            if remained_dist < 0.02:
                self.fnStop()
                self.current_nearby_sequence = self.NearbySequence.turn_right.value

        elif self.current_nearby_sequence == self.NearbySequence.turn_right.value:
            if self.is_triggered == False:
                self.is_triggered = True
                self.initial_robot_pose_theta = self.robot_2d_theta

            if self.initial_marker_pose_theta < 0.0:
                desired_angle_turn = -(math.pi / 2.0) + (self.robot_2d_theta - self.initial_robot_pose_theta)
            elif self.initial_marker_pose_theta > 0.0:
                desired_angle_turn = (math.pi / 2.0) + (self.robot_2d_theta - self.initial_robot_pose_theta)

            # rospy.loginfo("desired_angle_turn %f self.robot_2d_theta %f self.initial_robot_pose_theta %f"
            # , desired_angle_turn, self.robot_2d_theta, self.initial_robot_pose_theta)

            self.fnTurn(desired_angle_turn)

            if abs(desired_angle_turn) < 0.05:
                self.fnStop()
                self.current_nearby_sequence = self.NearbySequence.parking.value
                self.is_triggered = False
                return True

        return False

    def fnSeqParking(self):
        desired_angle_turn = math.atan2(self.marker_2d_pose_y - 0, self.marker_2d_pose_x - 0)
        self.fnTrackMarker(-desired_angle_turn)

        if abs(self.marker_2d_pose_x) < 0.22:
            self.fnStop()
            return True
        else:
            return False

    def fnStop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.pub_cmd_vel.publish(twist)

    def fnTurn(self, theta):
        Kp = 0.4
        angular_z = Kp * theta

        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = -angular_z
        self.pub_cmd_vel.publish(twist)

    def fnGoStraight(self):
        twist = Twist()
        twist.linear.x = 0.2
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.pub_cmd_vel.publish(twist)

    def fnTrackMarker(self, theta):
        Kp = 0.3

        angular_z = Kp * theta

        twist = Twist()
        twist.linear.x = 0.1
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = -angular_z
        self.pub_cmd_vel.publish(twist)

    def fnGet2DRobotPose(self, robot_odom_msg):
        quaternion = (robot_odom_msg.pose.pose.orientation.x, robot_odom_msg.pose.pose.orientation.y, robot_odom_msg.pose.pose.orientation.z, robot_odom_msg.pose.pose.orientation.w)
        theta = euler_from_quaternion(quaternion)[2]

        if theta < 0.0:
            theta = theta + np.pi * 2
        if theta > np.pi * 2:
            theta = theta - np.pi * 2

        pos_x = robot_odom_msg.pose.pose.position.x
        pos_y = robot_odom_msg.pose.pose.position.y

        return pos_x, pos_y, theta

    def quaternion_to_rotation_matrix(self, quaternion):
        x, y, z, w = quaternion
        rotation_matrix = np.array([[1-2*y**2-2*z**2, 2*x*y-2*z*w, 2*x*z+2*y*w],
                                    [2*x*y+2*z*w, 1-2*x**2-2*z**2, 2*y*z-2*x*w],
                                    [2*x*z-2*y*w, 2*y*z+2*x*w, 1-2*x**2-2*y**2]])
        return rotation_matrix

    def rotateOdom(self, odom):
        self.get_logger().info("odom {0}".format(odom.position))
        # position = np.array([odom.position.x, odom.position.y, odom.position.z])
        # orientation = np.array([odom.orientation.x, odom.orientation.y,
        #                         odom.orientation.z, odom.orientation.w])

        # rotation_matrix = self.quaternion_to_rotation_matrix(orientation)

        # rotated_position = np.dot(rotation_matrix, position)

        # rotated_orientation = orientation

        # rotated_odom = Pose()
        # rotated_odom.position.x = rotated_position[0]
        # rotated_odom.position.y = rotated_position[1]
        # rotated_odom.position.z = rotated_position[2]
        # rotated_odom.orientation.x = rotated_orientation[0]
        # rotated_odom.orientation.y = rotated_orientation[1]
        # rotated_odom.orientation.z = rotated_orientation[2]
        # rotated_odom.orientation.w = rotated_orientation[3]


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

        pose_matrix = [[odom.position.x],
                    [odom.position.y],
                    [odom.position.z]]
        rotated_odom_matrix = np.dot(rotation_matrix_z, np.dot(rotation_matrix_x, pose_matrix))

        orientation = [odom.orientation.x, odom.orientation.y, odom.orientation.z, odom.orientation.w]
        rotated_orientation = [0, 0, 0, 0]
        rotated_orientation[0] = cos_angle_x * cos_angle_z * orientation[0] - sin_angle_x * sin_angle_z * orientation[1]
        rotated_orientation[1] = sin_angle_x * cos_angle_z * orientation[0] + cos_angle_x * sin_angle_z * orientation[1]
        rotated_orientation[2] = cos_angle_x * sin_angle_z * orientation[0] + sin_angle_x * cos_angle_z * orientation[2]
        rotated_orientation[3] = cos_angle_x * cos_angle_z * orientation[3] - sin_angle_x * sin_angle_z * orientation[3]

        rotated_odom = Pose()
        rotated_odom.position.x = rotated_odom_matrix[0][0]
        rotated_odom.position.y = rotated_odom_matrix[1][0]
        rotated_odom.position.z = rotated_odom_matrix[2][0]
        rotated_odom.orientation.x = rotated_orientation[0]
        rotated_odom.orientation.y = rotated_orientation[1]
        rotated_odom.orientation.z = rotated_orientation[2]
        rotated_odom.orientation.w = rotated_orientation[3]

        return rotated_odom

    def fnGet2DMarkerPose(self, marker_odom_msg):
        # odom = self.rotateOdom(marker_odom_msg)
        odom = marker_odom_msg
        self.get_logger().info("rotation odom {0}".format(odom.position))
        quaternion = (
            odom.orientation.x,
            odom.orientation.y,
            odom.orientation.z,
            odom.orientation.w)
        theta = euler_from_quaternion(quaternion)[2]
        theta = theta + np.pi / 2.

        if theta < 0.0:
            theta = theta + np.pi * 2
        if theta > np.pi * 2:
            theta = theta - np.pi * 2

        pos_x = odom.position.x
        pos_y = odom.position.y

        return pos_x, pos_y, theta

    def fnCalcDistPoints(self, x1, x2, y1, y2):
        return math.sqrt((x1 - x2) ** 2. + (y1 - y2) ** 2.)

    def fnShutDown(self):
        self.get_logger().info("Shutting down. cmd_vel will be 0")
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.pub_cmd_vel.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = AutomaticParkingVision()

    rclpy.spin(node)

    node.fnShutDown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()