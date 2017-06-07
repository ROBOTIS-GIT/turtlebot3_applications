# /*******************************************************************************
# * Copyright (c) 2016, ROBOTIS CO., LTD.
# * All rights reserved.
# *
# * Redistribution and use in source and binary forms, with or without
# * modification, are permitted provided that the following conditions are met:
# *
# * * Redistributions of source code must retain the above copyright notice, this
# *   list of conditions and the following disclaimer.
# *
# * * Redistributions in binary form must reproduce the above copyright notice,
# *   this list of conditions and the following disclaimer in the documentation
# *   and/or other materials provided with the distribution.
# *
# * * Neither the name of ROBOTIS nor the names of its
# *   contributors may be used to endorse or promote products derived from
# *   this software without specific prior written permission.
# *
# * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# *******************************************************************************/

# /* Author: Chris, Ashe Kim */

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
