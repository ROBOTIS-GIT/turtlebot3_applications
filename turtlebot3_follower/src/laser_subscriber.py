#!/usr/bin/env python
import rospy
import pickle
from sensor_msgs.msg import LaserScan

class laser_subscriber:
    def __init__(self):
        self.laser_scan_data =[]
        self.comments=[]

        self.Subscriber()
#    def log_ranges(self, data):
#        rospy.loginfo('%s', data.ranges)

    def Subscriber(self):
        while not rospy.is_shutdown():
            comment = raw_input('Right a comment and pres Enter to continue..')
            self.comments.append(comment)

            self.msg = rospy.wait_for_message("/turtlebotA/scan_filtered", LaserScan)
            rospy.loginfo('%s', self.msg.ranges)
            self.laser_scan_data.append(self.msg)

            pickle.dump( self.comments, open( "comments_training_set", "ab")) # store range_msgs into a file
            pickle.dump( self.laser_scan_data, open( "laser_scan_data_training_set", "ab")) #store the hole laser_scan_data into a file

if __name__ == '__main__':

    rospy.init_node('laser_subscriber')
#    raw_input('test')
    try:
        laser = laser_subscriber()
    except rospy.ROSInterruptException:
        pass    #print("Shutting down")
