#!/usr/bin/env python
import rospy
import pickle
import os
import pandas as pd
import numpy as np
from pandas import Series, DataFrame
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

            self.msg = rospy.wait_for_message("/turtlebotA/scan_filtered", LaserScan)
            rospy.loginfo('%s', self.msg.ranges)
            self.laser_scan_data.append(self.msg)
            

            pickle.dump( self.comments, open( "add_comment", "wb"))
            pickle.dump( self.laser_scan_data, open( "add_data", "wb"))
            
#            
#            if os.path.isfile("add_comment") == True & os.path.isfile("add_data") == True:
#                comment = raw_input('Right a comment and pres Enter to continue..\n')
#                self.comments.append(comment)
#
#                self.msg = rospy.wait_for_message("/turtlebotA/scan_filtered", LaserScan)
#                self.laser_scan_data.append(self.msg)
##                
##                self.comments.append( f1.read(self.comments) +  self.comments)
##                self.laser_scan_data.append( f2.read(self.laser_scan_data) + self.laser_scan_data)
##                
##                f1.write(self.comments)
##                f2.write(self.laser_scan_data)
##                
##                f1.close()
##                f2.close()
#                pickle.load( open("add_comment", 'r+b' ))
#                pickle.load( open("add_data", 'r+b' ))
#                
#                self.comments = "add_comment" + self.comments
#                self.laser_scan_data = "add_data" + self.laser_scan_data
#                
#                pickle.dump(self.comments, "add_comment")
#                pickle.dump(self.laser_scan_data, "add_data")
#                pickle.dump( self.comments, open( "comments_training_set", 'wb'))
#                pickle.dump( self.laser_scan_data, open( "laser_scan_data_training_set", 'wb'))

if __name__ == '__main__':

    rospy.init_node('laser_subscriber')
    try:
        laser = laser_subscriber()
    except rospy.ROSInterruptException:
        pass    #print("Shutting down")
