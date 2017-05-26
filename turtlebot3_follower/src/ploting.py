#!/usr/bin/env python
import pickle
import rospy
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pandas import DataFrame
from sensor_msgs.msg import LaserScan
from sklearn.ensemble import RandomForestClassifier


if __name__ == '__main__':

    laser_data = pickle.load( open( "add_data", "rb"))
    comments = pickle.load( open( "add_comment", "rb" ))
    labels = {'30_0':0, '30_l':1, '30_r':2, '45_0':3, '45_l':4, '45_r':5, '15_0':6, 'empty':7}
                                        
#################################################### WORKING with 0-70 to 290- 360 #############################################

    data_range_training_set = []
    data_range_training = []
    data_intensity_training_set = []
    data_intensity_training = []
 
    for i in range(len(comments)):
        
        for x in range(70,-2,-1) + range(359, 289,-1):
            
            if   np.nan_to_num(laser_data[i].ranges[x]) != 0 :
                 data_range_training.append(np.nan_to_num(laser_data[i].ranges[x]))

            elif (x+1) in range(70,-2,-1) + range(359, 289,-1) and (x-1) in range(70,-2,-1) + range(359, 289,-1) and np.nan_to_num(laser_data[i].ranges[x]) == 0:
                 data_range_training.append((np.nan_to_num(laser_data[i].ranges[x+1])+np.nan_to_num(laser_data[i].ranges[x-1])/2))
                 
            else :
                 data_range_training.append(np.nan_to_num(laser_data[i].ranges[x]))
                 
            dd = np.array(data_range_training)
            dd[dd == 0] = 1
              
            dd = np.log(dd)
              
        for x in range(70,-2,-1) + range(359, 289,-1):
            
            if   np.nan_to_num(laser_data[i].intensities[x]) != 0 :
                 data_intensity_training.append(np.nan_to_num(laser_data[i].intensities[x]))
            
            elif (x+1) in range(70,-2,-1) + range(359, 289,-1) and (x-1) in range(70,-2,-1) + range(359, 289,-1) and np.nan_to_num(laser_data[i].intensities[x]) == 0:
                 data_intensity_training.append((np.nan_to_num(laser_data[i].intensities[x+1])+np.nan_to_num(laser_data[i].intensities[x-1])/2))             
        
            else :
                 data_intensity_training.append(np.nan_to_num(laser_data[i].intensities[x]))
                 
            cc = np.array(data_intensity_training)
            cc[cc == 0] = 1
              
            cc = np.log(cc)
                 
        data_range_training_set.append(dd)    
        data_intensity_training_set.append(cc) 

#################################################### PLOTTING FROM 0-70 290-360 ################################################
#        plt.figure(i)
#        
#        plt.subplot(211)
#        plt.axis([0, 140, -2.0, 0.0])
#        plt.plot(dd, 'b')
#        plt.title(comments[i])
#        plt.ylabel('Range')
#
#        plt.subplot(212)
#        plt.axis([0, 140, 0.0, 10])
#        plt.plot(cc, 'r') 
#        plt.ylabel('Intensitie')        

        data_range_training = []
        data_intensity_training = []
               
#################################################### Random Forest Fitting ####################################################

    Y=[]
    for i in range(len(comments)):
        Y.append(labels[comments[i]])

    clf = RandomForestClassifier(n_estimators = 10)
    clf.fit(data_range_training_set,Y)

    clf2 = RandomForestClassifier(n_estimators = 10)
    clf2.fit(data_intensity_training_set,Y)

    pickle.dump( clf, open( "clf", "wb"))
    pickle.dump( clf2, open( "clf2", "wb"))

