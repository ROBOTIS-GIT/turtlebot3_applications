#!/usr/bin/env python
import pickle
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import numpy as np
from sklearn.ensemble import RandomForestClassifier

if __name__ == '__main__':

    laser_data = pickle.load( open( "laser_scan_data_training_set", "rb"))
    comments = pickle.load( open( "comments_training_set", "rb"))
    print len(comments)
    labels = {'15cm_0' : 0, '15_0': 0, '30_0':1 , '30cm_0' : 1, '45cm_0' : 2,'45_0': 2, '15cm_45l' : 3, '15cm_l':3, '15_l':3 ,'30cm_45l' : 4, '30cm_l':4, '30_l':4, '45cm_45l':5, '45_l': 5, '45cm_l': 5, '15cm_45r':6, '15_r':6, '15cm_r':6, '30cm_45r':7, '30_r':7, '30cm_r':7,'45cm_45r':8, '45_r':8, 'empty':9, '45cm_r':8}
              
##################PLOTTING READINGS################################
#    for i in range(len(comments)):
#        plt.figure(i)
#        
#        plt.subplot(211)
#        plt.plot(laser_data[i].ranges, 'b')
#        plt.title(comments[i])
#        plt.ylabel('Range')
#        
#        plt.subplot(212) 
#        plt.plot(laser_data[i].intensities, 'r') 
#        plt.ylabel('Intensitie')
#    
#    for i in range(len(comments)):
#        if i !=0:
#            
#                plt.figure(len(comments)+i)
#                plt.subplot(211)
#                plt.title(comments[i] + " - empty space in front")
#                plt.plot(np.subtract(laser_data[i].ranges, laser_data[0].ranges), 'b')
#                plt.ylabel('Range')
#                plt.subplot(212) 
#                plt.plot(np.subtract(laser_data[i].intensities, laser_data[0].intensities), 'r') 
#                plt.ylabel('Intensitie')
#
#    print "Angle Min: %f " % laser_data[0].angle_min
#    print "Angle Max: %f " % laser_data[0].angle_max
#    print "Angle Increment: %f " %  laser_data[0].angle_increment
#                                        
#    print "Range Min: %f" % laser_data[0].range_min
#    print "Range max: %f" % laser_data[0].range_max
###############################################################

#    test = laser_data[0].ranges
#    test1 = laser_data[5].ranges
#    print comments[5]
#    a0 = []
#    a1 = []
#    for i in range(len(test)):
#    
#        if test[i]==0:
#            a0.append(i)
#        if test1[i]==0:
#            a1.append(i)
#      #      print i 
#      
#        if test1[i] < 0.4 and test1[i] > 0.05:
#            print i
#    a = [x for x in a1 if x not in a0]
#    print a
#    print [test[x] for x in a]                                    
                                        
####### WORKING with 0 to 60 300- 360####################
    data_range_training_set = []
    data_range_training = []
 
    for i in range(len(comments)):
        for x in range(59,-1,-1) + range(359, 299,-1):
            if laser_data[i].ranges[x] != 0 :
                data_range_training.append(np.log(laser_data[i].ranges[x])) ##trying logarithm to get better scale on range

            elif (x+1) in range(59,-1,-1) + range(359, 299,-1) and (x-1) in range(59,-1,-1) + range(359, 299,-1) and laser_data[i].ranges[(x+1)] != 0 and laser_data[i].ranges[(x-1)] != 0:
                data_range_training.append(np.log((laser_data[i].ranges[x+1]+laser_data[i].ranges[x-1])/2)) ## manipulating zeros

            else:
                data_range_training.append(np.log(3.5))             

        data_range_training_set.append(data_range_training)      

#############PLOTTING FROM 0-60 300-360
#        plt.figure(i)
#       # plt.plot(np.log(test4))
##        plt.subplot(211)
#        plt.plot(data_range_training, 'b')
#        plt.title(comments[i])
#        plt.ylabel('Range')
        data_range_training = []
#    
#        plt.subplot(212) 
#        plt.plot(data_intensitie_training, 'r') 
#        plt.ylabel('Intensitie')
        #        
##################################################### Random Forest Fitting ####################

    Y=[]
    for i in range(len(comments)):
        Y.append(labels[comments[i]])

    clf = RandomForestClassifier(n_estimators = 10)
    clf.fit(data_range_training_set,Y)

    pickle.dump( clf, open( "clf", "ab")) # store range_msgs into a file

#
###########################Loading training data and testing ##############################
#    testing_set = pickle.load( open( "laser_scan_data_training_set", "rb"))
#    testing_labels = pickle.load( open( "comments_training_set", "rb")) 
#    data_range_testing = []
#    data_testing = []
#    data_range_testing_set = []
#    print testing_set[0].ranges
#    data_intensitie_testing = []
#    
#    for i in range(len(testing_labels)-7):
#        data_testing.append(testing_set[i].ranges)
#    print len(data_testing)
# 
#    for i in range(len(testing_labels)-7):
#        for x in range(59,-1,-1) + range(359, 299,-1):
#            if testing_set[i].ranges[x] != 0 :
#                data_range_testing.append(np.log(testing_set[i].ranges[x])) ##trying logarithm to get better scale on range
##                #data_intensitie_testing.append(np.log(laser_data[i].intensities[x]/1000))
#            #else:
#            elif (x+1) in range(59,-1,-1) + range(359, 299,-1) and (x-1) in range(59,-1,-1) + range(359, 299,-1) and testing_set[i].ranges[(x+1)] != 0 and testing_set[i].ranges[(x-1)] != 0:
#                data_range_testing.append(np.log((testing_set[i].ranges[x+1]+testing_set[i].ranges[x-1])/2)) ## manipulating zeros
#            else:
#                data_range_testing.append(np.log(3.5))
#        data_range_testing_set.append(data_range_testing)
#        data_range_testing = []
#          
#    
##    print comments
#    Yy = []
#    print clf.predict( data_range_testing_set)
#    for i in range(len(testing_labels)-7):
#        Yy.append(labels[testing_labels[i]])
#    print Yy
#        
#    print clf.score(data_range_testing_set, Yy)
#    #print clf1.get_params()
