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

    laser_data = pickle.load( open( "add_data2", "rb"))
    comments = pickle.load( open( "add_comment2", "rb"))
    labels = {'15_0':0, '30_0':1 , '45_0':2, '15_l':3 , '30_l':4, '45cm_l':5, '15_r':6, '30_r':7, '45_r':8, 'empty':9}

                                        
####### WORKING with 0 to 60 300- 360####################

    data_range_training_set = []
    data_range_training = []
    data_intensity_training_set = []
    data_intensity_training = []
 
    for i in range(len(comments)):
        for x in range(59,-1,-1) + range(359, 299,-1):
            if laser_data[i].ranges[x] != 0 :
                data_range_training.append(np.log(laser_data[i].ranges[x])) ##trying logarithm to get better scale on range
                data_intensity_training.append(laser_data[i].intensities[x]) ##trying logarithm to get better scale on range

            elif (x+1) in range(59,-1,-1) + range(359, 299,-1) and (x-1) in range(59,-1,-1) + range(359, 299,-1) and laser_data[i].ranges[(x+1)] == 0 and laser_data[i].ranges[(x-1)] == 0:
                data_range_training.append(np.log((laser_data[i].ranges[x+1]+laser_data[i].ranges[x-1])/2)) ## manipulating zeros

            else:
                data_range_training.append(np.log(3.5))             
            
        data_range_training = np.nan_to_num(data_range_training)
        data_intensity_training = np.nan_to_num(data_intensity_training)

        data_range_training_set.append(data_range_training)    
        data_intensity_training_set.append(data_intensity_training) 

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
        data = {'range':data_range_training, 'intensity':data_intensity_training}
        frame = DataFrame(data)
        frame = frame.replace(np.nan, 0.0)
#        print('frame', frame)

        data_range_training = []
        data_intensity_training = []
               
##################################################### Random Forest Fitting ####################

    Y=[]
    for i in range(len(comments)):
        Y.append(labels[comments[i]])

    clf = RandomForestClassifier(n_estimators = 10)
    clf.fit(data_range_training_set,Y)

    pickle.dump( clf, open( "clf", "ab")) # store range_msgs into a file

