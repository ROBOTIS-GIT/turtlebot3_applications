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
        self.labels = {'15cm_0' : 0, '30cm_0' : 1, '45cm_0' : 2, '15cm_45l' : 3, '30cm_45l' : 4, '45cm_45l':5, '15cm_45r':6, '30cm_45r':7, '45cm_45r':8, 'empty':9}
        rospy.loginfo('Tree initialized')
        self.follow()

    def laser_scan(self):    ## estimate position
        data_test=[]
        data_test_set=[]
        self.msg = rospy.wait_for_message("/turtlebotA/scan", LaserScan)

        ### manipulating the data for better scale on ranges
        for x in range(59,-1,-1) + range(359, 299,-1):
            if self.msg.ranges[x] != 0 :
                data_test.append(np.log(self.msg.ranges[x]))
            elif (x+1) in range(59,-1,-1) + range(359, 299,-1) and (x-1) in range(59,-1,-1) + range(359, 299,-1) and self.msg.ranges[(x+1)] != 0 and self.msg.ranges[(x-1)] != 0:
                data_test.append(np.log((self.msg.ranges[x+1]+self.msg.ranges[x-1])/2))
            else:
                data_test.append(np.log(3.5))
        data_test_set.append(data_test)

        return [x for (x , y) in self.labels.iteritems() if y == self.clf.predict(data_test_set) ] ## Predict the position

    def follow(self):
        while not rospy.is_shutdown():
            x = self.laser_scan()
            twist = Twist()
            ## Do something according to each position##
            if x == ['15cm_0']:
                twist.linear.x  = 0.0;	twist.angular.z = 0.0;
            elif x== ['30cm_0']:
                twist.linear.x  = 0.1;	twist.angular.z = 0.0;
            elif x== ['45cm_0']:
                twist.linear.x  = 0.15;	twist.angular.z = 0.0;
            elif x== ['15cm_45l']:
                twist.linear.x  = 0.0;	twist.angular.z = 0.4;
            elif x== ['15cm_45r']:
                twist.linear.x  = 0.0;	twist.angular.z = -0.4;
            elif x== ['30cm_45r']:
                twist.linear.x  = 0.05;	twist.angular.z = -0.30;
            elif x== ['30cm_45l']:
                twist.linear.x  = 0.05;	twist.angular.z = 0.30;
            elif x== ['45cm_45r']:
                twist.linear.x  = 0.08;	twist.angular.z = -0.27;
            elif x== ['45cm_45l']:
                twist.linear.x  = 0.08;	twist.angular.z = 0.27;
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
