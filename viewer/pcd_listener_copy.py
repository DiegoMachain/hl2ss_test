#!/usr/bin/env python
#Example of listener of point cloud

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import ros_numpy
import open3d as o3d
import numpy as np

import Ditto_pipeline


def readPCD(cloud_npy):
    

    return o3dpc

class DittoManager():

    def __init__(self):
        self.src_pcd_list = []
        self.src_pc_list = []
        self.dst_pcd_list = []

        self.scale = 0.
        self.center = 0.

    def appendSRC(self, pcd):
        self.src_pcd_list.append(pcd)

        #Append for the scaling of the point clouds
        pc = np.asarray(pcd.points)
        pc_idx = np.random.randint(0, pc.shape[0], size=(2048, ))
        pc = pc[pc_idx]
        self.src_pc_list.append(pc)
        

    def appendDST(self, pcd):
        self.dst_pcd_list.append(pcd)


    def scalePCD(self):
        src_fused_pc = np.concatenate(self.src_pc_list, axis=0)
        self.center = (np.min(src_fused_pc, 0) + np.max(src_fused_pc, 0)) / 2
        self.scale = (np.max(src_fused_pc, 0) - np.min(src_fused_pc, 0)).max()
        self.scale *= 1.1

    def transformPCDSRC(self):

        for k in range(len(self.src_pcd_list)):
            center_transform = np.eye(4)
            center_transform[:3, 3] = -self.center
            self.src_pcd_list[k].transform(center_transform)
            self.src_pcd_list[k].scale(1 / self.scale, np.zeros((3, 1)))
            
            #Rotation
            R = self.src_pcd_list[k].get_rotation_matrix_from_xyz((np.pi / 2, np.pi/2, 0))
            self.src_pcd_list[k].rotate(R, center=(0, 0, 0))
            
    def transformPCDDST(self):

        for k in range(len(self.dst_pcd_list)):
            center_transform = np.eye(4)
            center_transform[:3, 3] = -self.center
            self.dst_pcd_list[k].transform(center_transform)
            self.dst_pcd_list[k].scale(1 / self.scale, np.zeros((3, 1)))
            
            #Rotation
            R = self.dst_pcd_list[k].get_rotation_matrix_from_xyz((np.pi / 2, np.pi/2, 0))
            self.dst_pcd_list[k].rotate(R, center=(0, 0, 0))


    def callDitto(self):

        Ditto_pipeline.Ditto(self.src_pcd_list, self.dst_pcd_list)

    #Function to reset the manager 
    def reset(self):
        self.src_pcd_list = []
        self.src_pc_list = []
        self.dst_pcd_list = []

        self.scale = 0.
        self.center = 0.


def callback(data):

    field_names = [field.name for field in data.fields]
    is_rgb = 'rgb' in field_names

    cloud_array = ros_numpy.point_cloud2.pointcloud2_to_array(data).ravel()
    if is_rgb:
        cloud_npy = np.zeros(cloud_array.shape + (4,), dtype=np.float)
    else: 
        cloud_npy = np.zeros(cloud_array.shape + (3,), dtype=np.float)

    cloud_npy[...,0] = cloud_array['x']
    cloud_npy[...,1] = cloud_array['y']
    cloud_npy[...,2] = cloud_array['z']
    o3dpc = o3d.geometry.PointCloud()

    if len(np.shape(cloud_npy)) == 3:
        cloud_npy = np.reshape(cloud_npy[:, :, :3], [-1, 3], 'F')
    o3dpc.points = o3d.utility.Vector3dVector(cloud_npy[:, :3])

    if data.header.frame_id == "before":
        ditto.appendSRC(o3dpc)
    elif data.header.frame_id == "after":
        ditto.appendDST(o3dpc)

    elif data.header.frame_id == "end":
        #scale the pointclouds to call ditto with them
        ditto.scalePCD()
        ditto.transformPCDSRC()
        ditto.transformPCDDST()

        #Call Ditto
        ditto.callDitto()

        ditto.reset()


    print("Point cloud = ", np.asarray(o3dpc.points))


#Initiate DittoManager
ditto = DittoManager()

rospy.init_node('listener', anonymous=True)

rospy.Subscriber("point_cloud2", PointCloud2, callback)

rospy.spin()
