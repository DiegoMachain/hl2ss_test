#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray


rospy.init_node('HoloLens_listener', anonymous=True)

def callback(data):
    #rospy.loginfo("Hololens Data = ", data.data)

    print("HoloLens data = ", data.data)




rospy.Subscriber("parameters", Float64MultiArray, callback)

rospy.spin()

