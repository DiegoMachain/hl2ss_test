#!/usr/bin/env python

from __future__ import print_function

import random
import rospy
import numpy as np

from unity_robotics_demo_msgs.srv import PositionService, PositionServiceResponse


def new_position(req):
    
    #req.input.rot_x = random.uniform(-3.0, 3.0)
    #req.input.rot_y = random.uniform(-1.5, 1.5)
    #req.input.rot_z = random.uniform(-3.0, 3.0)

    req.input.pos_x = 0
    req.input.pos_y = random.uniform(0, 0.1)
    req.input.pos_z = 0.7

    print("Request: \n{}".format(req.input))
    

    return PositionServiceResponse(req.input)



def translate_position_server():
    rospy.init_node('position_server')
    s = rospy.Service('pos_srv', PositionService, new_position)
    print("Ready to move cubes!")
    rospy.spin()


if __name__ == "__main__":
    translate_position_server()
