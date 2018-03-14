#!/usr/bin/env python
import rospy
import pickle
import os
import numpy as np
from sensor_msgs.msg import LaserScan

class laser_subscriber:
    def __init__(self):
        self.laser_scan_data =[]
        self.comments=[]

        self.Subscriber()

    def Subscriber(self):
        while not rospy.is_shutdown():
            comment = raw_input('Right a comment and pres Enter to continue..\n')
            self.comments.append(comment)

            self.msg = rospy.wait_for_message("/scan_filtered", LaserScan)
            rospy.loginfo('%s', self.msg.ranges)
            self.laser_scan_data.append(self.msg)

            pickle.dump( self.comments, open( "add_comment", "wb"))
            pickle.dump( self.laser_scan_data, open( "add_data", "wb"))

if __name__ == '__main__':

    rospy.init_node('laser_subscriber')
    try:
        laser = laser_subscriber()
    except rospy.ROSInterruptException:
        pass    #print("Shutting down")
