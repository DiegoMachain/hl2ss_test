#!/usr/bin/env python
#Example publisher for a point cloud
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Transform, TransformStamped, Vector3
import open3d as o3d
from std_msgs.msg import Header, String

import rospy
import struct
import numpy as np

import crop_pcl


#Initiate the nodes

rospy.init_node("create_cloud_xyzrgb")

pub = rospy.Publisher("point_cloud2", PointCloud2, queue_size=2)


fields = [PointField('x', 0, PointField.FLOAT32, 1),
          PointField('y', 4, PointField.FLOAT32, 1),
          PointField('z', 8, PointField.FLOAT32, 1),
          PointField('rgb', 12, PointField.UINT32, 0),
          PointField('state', 16, PointField.UINT32,0),
          #PointField('rgba', 12, PointField.UINT32, 1),
          ]

#pub = rospy.Publisher("point_cloud2", PointCloud2, queue_size=2)

header = Header()
header.frame_id = "before"

#Path for the examples
path = "/home/vboxuser/Desktop/Semester_Project/Ditto_ROS/hl2ss_test/viewer/output/run1/"

bbox = False

#Send the before and after the interaction and crop them


for k in range(3):
    
    pcd = o3d.io.read_point_cloud("%sworldpcd_%s.ply" %(path, k))


    if not bbox:
        points = crop_pcl.pick_points(pcd)
        picked_points = np.asarray(pcd.points)[points]
        picked_points = o3d.utility.Vector3dVector(picked_points)
        bbox = o3d.geometry.OrientedBoundingBox.create_from_points(picked_points)
        #bbox = o3d.geometry.AxisAlignedBoundingBox.create_from_points(picked_points)

    #Draw a box to cut the object we want
    pcd_2 = pcd.crop(bbox)
    _ = crop_pcl.pick_points(pcd_2)

    pc2 = point_cloud2.create_cloud(header, fields, np.asarray(pcd_2.points))

    pc2.header.stamp = rospy.Time.now()
    pub.publish(pc2)

#Publish the after


#bbox = False
#Send the before and after the interaction and crop them

header = Header()
header.frame_id = "after"


for k in range(70, 73):
    
    pcd = o3d.io.read_point_cloud("%sworldpcd_%s.ply" %(path, k))

    #Crop the point cloud

    if not bbox:
        points = crop_pcl.pick_points(pcd)
        picked_points = np.asarray(pcd.points)[points]
        picked_points = o3d.utility.Vector3dVector(picked_points)
        #bbox = o3d.geometry.OrientedBoundingBox.create_from_points(picked_points)
        bbox = o3d.geometry.AxisAlignedBoundingBox.create_from_points(picked_points)

    #Crop the pointcloud with the box
    pcd_2 = pcd.crop(bbox)
    _ = crop_pcl.pick_points(pcd_2)

    pc2 = point_cloud2.create_cloud(header, fields, np.asarray(pcd_2.points))

    pc2.header.stamp = rospy.Time.now()
    pub.publish(pc2)

#end
#Unnecesary publish, it's only to realize that we are done with the messages
header = Header()
header.frame_id = "end"

pc2 = point_cloud2.create_cloud(header, fields, np.asarray(pcd_2.points))

pc2.header.stamp = rospy.Time.now()
pub.publish(pc2)

