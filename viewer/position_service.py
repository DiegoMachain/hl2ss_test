#!/usr/bin/env python

from __future__ import print_function

import random
import rospy
import numpy as np
import math



from unity_robotics_demo_msgs.srv import PositionService, PositionServiceResponse


def quat_from_matrix(mat):
    q = np.empty((4,), dtype = np.float64)
    M = np.array(mat, dtype = np.float64, copy=False)[:4,:4]
    t = np.trace(M)

    if t > M[3, 3]:
        q[3] = t
        q[2] = M[1,0] - M[0,1]
        q[1] = M[0,2] - M[2,0]
        q[0] = M[2,1] - M[1,2]
    else:
        i,j,k = 0, 1, 2
        if M[1 ,1] > M[0,0]:
            i,j,k = 1,2,0
        if M[2,2] > M[i,i]:
            i,j,k = 2,0,1
        t = M[i,i] - (M[j,j] + M[k,k]) + M[3,3]
        q[i] = t
        q[j] = M[i,j] + M[j,i]
        q[k] = M[k,i] + M[i,k]
        q[3] = M[k,j] - M[j,k]
    q *= 0.5 / math.sqrt(t * M[3,3])
    return q

def new_position(req):
    
    

    req.input.pos_x = 0
    req.input.pos_y = random.uniform(0, 0.1)
    req.input.pos_z = -0.6

    print("Request: \n{}".format(req.input))
    
    ang = np.pi/2.
    #X
    quat = np.array([[1,0,0,0],[0,np.cos(ang), -np.sin(ang),0],[0,np.sin(ang),np.cos(ang),0],[0,0,0,1]])
    #Y
    #quat = np.array([[np.cos(ang),0,np.sin(ang),0],[0, 1, 0,0],[-np.sin(ang), 0, np.cos(ang),0],[0,0,0,1]])
    #Z
    #quat = np.array([[np.cos(ang), -np.sin(ang), 0, 0],[np.sin(ang), np.cos(ang), 0,0],[0, 0, 1,0],[0,0,0,1]])
    print(quat)
    quat = quat_from_matrix(quat)
    print(quat)
    req.input.rot_x = quat[0]
    req.input.rot_y = quat[1]
    req.input.rot_z = quat[2]
    req.input.rot_w = quat[3]

    print(quat)

    return PositionServiceResponse(req.input)



def translate_position_server():
    rospy.init_node('position_server')
    s = rospy.Service('pos_srv', PositionService, new_position)
    print("Ready to move cubes!")
    rospy.spin()


if __name__ == "__main__":
    translate_position_server()
