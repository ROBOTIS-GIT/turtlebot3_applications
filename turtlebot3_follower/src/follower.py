#!/usr/bin/env python
import rospy
import pickle
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import pickle
from sklearn.ensemble import RandomForestClassifier
import numpy as np


class follower:
    def __init__(self):
        rospy.loginfo('Follower node initialized')
        self.pub = rospy.Publisher('/turtlebotA/cmd_vel', Twist, queue_size = 1)
        self.clf = pickle.load( open( "clf", "rb")) ## Load classifier
        self.labels = {'30_0':0, '30_l':1, '30_r':2, '45_0':3, '45_l':4, '45_r':5,'15_0':6, 'empty':7}
        rospy.loginfo('Tree initialized')
        self.follow()

    def laser_scan(self):    ## estimate position
        data_test=[]
        data_test_set=[]
        self.msg = rospy.wait_for_message("/turtlebotA/scan_filtered", LaserScan)

        for x in range(70,-2,-1) + range(359, 289,-1):
            
            if   np.nan_to_num( self.msg.ranges[x] ) != 0 :
                 data_test.append(np.nan_to_num(self.msg.ranges[x]))

            elif (x+1) in range(70,-2,-1) + range(359, 289,-1) and (x-1) in range(70,-2,-1) + range(359, 289,-1) and np.nan_to_num(self.msg.ranges[x]) == 0:
                 data_test.append((np.nan_to_num(self.msg.ranges[x+1])+np.nan_to_num(self.msg.ranges[x-1]))/2)
                 
            else :
                 data_test.append(np.nan_to_num(self.msg.ranges[x]))

        data_test_set.append(data_test)

        return [x for (x , y) in self.labels.iteritems() if y == self.clf.predict(data_test_set) ] ## Predict the position

    def follow(self):
        while not rospy.is_shutdown():
            x = self.laser_scan()
            twist = Twist()
            ## Do something according to each position##
            if  x == ['30_0']:
                twist.linear.x  = 0.05; 	twist.angular.z = 0.0;
            elif x== ['30_l']:
                twist.linear.x  = 0.04; 	twist.angular.z = 0.3;
            elif x== ['30_r']:
                twist.linear.x  = 0.04; 	twist.angular.z = -0.3;
            elif x== ['45_0']:
                twist.linear.x  = 0.08; 	twist.angular.z = 0.0;
            elif x== ['45_l']:
                twist.linear.x  = 0.07; 	twist.angular.z = 0.4;
            elif x== ['45_r']:
                twist.linear.x  = 0.07;	twist.angular.z = -0.4;
            elif x== ['15_0']:
                 twist.linear.x = 0.0; 	twist.angular.z = 0.0; 
            elif x== ['empty']:
                twist.linear.x  = 0.0; 	twist.angular.z = 0.0;
            else:
                twist.linear.x  = 0.0;	twist.angular.z = 0.0;
            
            self.pub.publish(twist)

def main():

    rospy.init_node('follower', anonymous=True)

    try:
        follow = follower()
    except rospy.ROSInterruptException:
        pass    #print("Shutting down")


if __name__ == '__main__':
    main()
