#!/usr/bin/env python

from __future__ import print_function

import random
import rospy
import numpy as np

from unity_robotics_demo_msgs.srv import PositionService, PositionServiceResponse


def new_position(req):
    print("Request: \n{}".format(req.input))
    req.input.rot_x = random.uniform(-1.0, 1.0)
    req.input.rot_y = random.uniform(-1.0, 1.0)
    req.input.rot_z = random.uniform(-1.0, 1.0)

    #req.input.pos_x = random.uniform(-1.0, 1.0)
    #req.input.pos_y = random.uniform(-1.0, 1.0)
    #req.input.pos_z = random.uniform(-1.0, 1.0)

    pivot = np.array([-0.06263177,  0.05984291, -0.16308576])
    vec = np.array([-0.61942077, -0.7850084,  -0.00417794])
    scale = 0.6441019214689732
    center = np.array([ 0.33128244, -0.33428816, -0.59747717])
    

    return PositionServiceResponse(req.input)



def translate_position_server():
    rospy.init_node('position_server')
    s = rospy.Service('pos_srv', PositionService, new_position)
    print("Ready to move cubes!")
    rospy.spin()


if __name__ == "__main__":
    translate_position_server()
