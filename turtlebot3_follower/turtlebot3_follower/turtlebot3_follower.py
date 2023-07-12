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

import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan

import pickle
import numpy as np

class follower(Node):

    def __init__(self):
        super().__init__('follower')

        package_dir = get_package_share_directory('turtlebot3_follower')
        self.config_dir = os.path.join(package_dir, 'config')
        # self.config_dir = self.config_dir.replace('nodes', 'config')
        self.laser_scan_data =[]
        self.comments=[]

        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            qos_profile=QoSProfile(depth=10))

        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self._scan_callback,
            qos_profile=qos_profile_sensor_data)

        self.clf = pickle.load(open(self.config_dir + '/clf', "rb"))
        self.clf2 = pickle.load(open(self.config_dir + '/clf2', "rb"))
        self.labels = {'30_0':0, '30_l':1, '30_r':2, '45_0':3, '45_l':4, '45_r':5,'15_0':6, 'empty':7}
        self.get_logger().info('Tree initialized')

        self.run_timer = self.create_timer(0.1, self.follow)

    def check_people(self):
        laser_data=[]
        laser_data_set=[]
        result=[]
        ret = 0
        self.msg = rospy.wait_for_message("scan_filtered", LaserScan)

        for i in range(70,-2,-1) + range(359, 289,-1):

            if   np.nan_to_num( self.msg.intensities[i] ) != 0 :
                 laser_data.append(np.nan_to_num(self.msg.intensities[i]))

            elif (i+1) in range(70,-2,-1) + range(359, 289,-1) and (i-1) in range(70,-2,-1) + range(359, 289,-1) and np.nan_to_num(self.msg.intensities[i]) == 0:
                 laser_data.append((np.nan_to_num(self.msg.intensities[i+1])+np.nan_to_num(self.msg.intensities[i-1]))/2)

            else :
                 laser_data.append(np.nan_to_num(self.msg.intensities[i]))

        laser_data_set.append(laser_data)

        [x for (x , y) in self.labels.iteritems() if y == self.clf2.predict(laser_data_set) ] ## Predict the position

        if result == ['empty']:
            ret = 0

        else:
            ret = 1

        return ret

    def laser_scan(self):
        data_test=[]
        data_test_set=[]
        self.msg = rospy.wait_for_message("scan_filtered", LaserScan)

        for i in range(70,-2,-1) + range(359, 289,-1):

            if   np.nan_to_num( self.msg.ranges[i] ) != 0 :
                 data_test.append(np.nan_to_num(self.msg.ranges[i]))

            elif (i+1) in range(70,-2,-1) + range(359, 289,-1) and (i-1) in range(70,-2,-1) + range(359, 289,-1) and np.nan_to_num(self.msg.ranges[i]) == 0:
                 data_test.append((np.nan_to_num(self.msg.ranges[i+1])+np.nan_to_num(self.msg.ranges[i-1]))/2)

            else :
                 data_test.append(np.nan_to_num(self.msg.ranges[i]))

        data_test_set.append(data_test)

        return [x for (x , y) in self.labels.iteritems() if y == self.clf.predict(data_test_set) ]

    def follow(self):
        check = self.check_people()
        if  check == 1:
            x = self.laser_scan()
            twist = Twist()
            ## Do something according to each position##
            if  x == ['30_0']:
                twist.linear.x  = 0.13;      	twist.angular.z = 0.0;
            elif x== ['30_l']:
                twist.linear.x  = 0.10; 		twist.angular.z = 0.4;
            elif x== ['30_r']:
                twist.linear.x  = 0.10; 		twist.angular.z = -0.4;
            elif x== ['45_0']:
                twist.linear.x  = 0.13;      	twist.angular.z = 0.0;
            elif x== ['45_l']:
                twist.linear.x  = 0.10; 		twist.angular.z = 0.3;
            elif x== ['45_r']:
                twist.linear.x  = 0.10; 		twist.angular.z = -0.3;
            elif x== ['15_0']:
                twist.linear.x  = 0.0;	      	twist.angular.z = 0.0;
            elif x== ['empty']:
                twist.linear.x  = 0.0;	 	    twist.angular.z = 0.0;
            else:
                twist.linear.x  = 0.0;		    twist.angular.z = 0.0;

            self.pub.publish(twist)

        elif check == 0:
            x = self.laser_scan()
            twist = Twist()

            if x== ['empty']:
                twist.linear.x  = 0.0;		twist.angular.z = 0.0;

            else:
                twist.linear.x  = 0.0; 		twist.angular.z = 0.4;

            self.pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    follow = follower()

    try:
        rclpy.spin(follow)
    except KeyboardInterrupt:
        pass
    finally:
        follow.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
