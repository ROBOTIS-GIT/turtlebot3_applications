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

import rospy
import os
import pickle
from sensor_msgs.msg import LaserScan

class laser_subscriber:
    def __init__(self):
        self.config_dir = os.path.join(os.path.dirname(__file__))
        self.config_dir = self.config_dir.replace('nodes', 'config')
        self.laser_scan_data =[]
        self.comments=[]
        self.Subscriber()

    def Subscriber(self):
        while not rospy.is_shutdown():
            comment = raw_input('Right a comment and pres Enter to continue..\n')
            self.comments.append(comment)

            self.msg = rospy.wait_for_message("scan_filtered", LaserScan)
            rospy.loginfo('%s', self.msg.ranges)
            self.laser_scan_data.append(self.msg)

            pickle.dump(self.comments, open(self.config_dir + '/add_comment', 'wb'))
            pickle.dump(self.laser_scan_data, open(self.config_dir + '/add_data', 'wb'))

if __name__ == '__main__':

    rospy.init_node('laser_subscriber')
    try:
        laser = laser_subscriber()
    except rospy.ROSInterruptException:
        pass
