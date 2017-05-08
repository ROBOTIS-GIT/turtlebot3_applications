#!/usr/bin/env python
import pickle
import rospy
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pandas import Series, DataFrame
from sensor_msgs.msg import LaserScan
from sklearn.ensemble import RandomForestClassifier


if __name__ == '__main__':

    laser_data = pickle.load( open( "laser_scan_data_training_set", 'rb'))
    comments = pickle.load( open( "comments_training_set", 'rb'))
    print len(comments)
    labels = {'15cm_0' : 0, '15_0': 0, '30_0':1 , '30cm_0' : 1, '45cm_0' : 2,'45_0': 2, '15cm_45l' : 3, '15cm_l':3, '15_l':3 ,'30cm_45l' : 4, '30cm_l':4, '30_l':4, '45cm_45l':5, '45_l': 5, '45cm_l': 5, '15cm_45r':6, '15_r':6, '15cm_r':6, '30cm_45r':7, '30_r':7, '30cm_r':7,'45cm_45r':8, '45_r':8, 'empty':9, '45cm_r':8}
              
##################PLOTTING READINGS################################
#    print "Angle Min: %f " % laser_data[0].angle_min
#    print "Angle Max: %f " % laser_data[0].angle_max
#    print "Angle Increment: %f " %  laser_data[0].angle_increment
#                                        
#    print "Range Min: %f" % laser_data[0].range_min
#    print "Range max: %f" % laser_data[0].range_max
                                        
####### WORKING with 0 to 60 300- 360####################
#
    data_range_training_set = []
    data_range_training = []
    data_intensity_training_set = []
    data_intensity_training = []
 
    for i in range(len(comments)):
        for x in range(59,-1,-1) + range(359, 299,-1):
            if laser_data[i].ranges[x] != 0 :
                data_range_training.append(np.log(laser_data[i].ranges[x])) ##trying logarithm to get better scale on range
                data_intensity_training.append(laser_data[i].intensities[x] ) ##trying logarithm to get better scale on range

            elif (x+1) in range(59,-1,-1) + range(359, 299,-1) and (x-1) in range(59,-1,-1) + range(359, 299,-1) and laser_data[i].ranges[(x+1)] != 0 and laser_data[i].ranges[(x-1)] != 0:
                data_range_training.append(np.log((laser_data[i].ranges[x+1]+laser_data[i].ranges[x-1])/2)) ## manipulating zeros

            else:
                data_range_training.append(np.log(3.5))             
        
        data_range_training = Series(data_range_training)    
        data_intensity_training = Series(data_intensity_training)
        
        data = {'range':data_range_training, 'intensity':data_intensity_training}
        frame = DataFrame(data)
        frame.replace(np.nan, '')
        print(frame)

        data_range_training_set = Series(data_range_training)    
        data_intensity_training_set = Series(data_intensity_training) 
        
#        print(data_range_training)
#        print(data_intensity_training)

#############PLOTTING FROM 0-60 300-360
        plt.figure(i)
        
        plt.subplot(211)
        plt.axis([0, 120, -2.0, 0.0])
        plt.plot(data_range_training, 'b')
        plt.title(comments[i])
        plt.ylabel('Range')
    
        plt.subplot(212)
        plt.plot(data_intensity_training, 'r') 
        plt.ylabel('Intensitie')
        
        data_intensity_training = data_intensity_training[30:90]
        data_range_training = data_range_training[30:90]
#        data = {'range':data_range_training, 'intensity':data_intensity_training}
#        frame = DataFrame(data)
#        df.replace(np.nan, 5)
        
        
        data_range_training = []
        data_intensity_training = []
               
##################################################### Random Forest Fitting ####################
#
#    Y=[]
#    for i in range(len(comments)):
#        Y.append(labels[comments[i]])
#
#    clf = RandomForestClassifier(n_estimators = 10)
#    clf.fit(data_range_training_set,Y)
#
#    pickle.dump( clf, open( "clf", "ab")) # store range_msgs into a file
#
