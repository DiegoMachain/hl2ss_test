import numpy as np
import cv2

import rospy
import rosbag
from std_msgs.msg import Header
from sensor_msgs.msg import PointField
import sensor_msgs.point_cloud2 as pcl2
from geometry_msgs.msg import Pose
import tf
import open3d as o3d

poses = []
poses_ts_s = []
poses_ts_ns = []
pcls = []
pcls_ts_s = []
pcls_ts_ns = []

c = 0
n = 100
open3d_pcls = []

# Init ROS
rospy.init_node('RosbagRewriter', anonymous=False)
tl = tf.TransformListener()

def transform_data(pose_msg, pcl_msg):
  points = []
  for p in pcl2.read_points(pcl_msg, field_names = ("x", "y", "z"), skip_nans=True):
    points.append(np.asarray([p[0],p[1],p[2],1]))
    # print(" x : %f  y: %f  z: %f" %(p[0],p[1],p[2]))
  # print("Pcl msg has %s points" % len(points))

  # Generate transformation matrix 
  translation = (pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z)
  rotation = (pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w)
  hololens_to_world = tl.fromTranslationRotation(translation, rotation)
  # world_to_hololens = tl.fromTranslationRotation(translation, rotation)
  # hololens_to_world = np.linalg.inv(world_to_hololens)
  print("Trafo %s: %s" % (c, hololens_to_world))

  # Transform each point in the pointcloud
  transformed_points = []
  count = 0
  for p in points:
    # if c == 0:
    #   print("Point before: %s" % p)
    t_p = np.dot(hololens_to_world, p)
    # t_p[2] = t_p[2] * -1
    # if c == 0:
    #   print("Point after: %s" % t_p)
    transformed_points.append(t_p)
    # c += 1

  # Generate and return new pointcloud2 message
  header = Header()
  header.stamp = msg.header.stamp
  header.frame_id = "/hololens"
  # Points have to be filled exactly like this to enable conversion via pcl
  fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1)]
  transformed_points = np.asarray(transformed_points)
  # print("Shape: ", transformed_points.shape)
  # print("First entry: ", transformed_points[0,:])
  # print("First entry: ", transformed_points[0,:3])
  pcl_msg = pcl2.create_cloud(header, fields, transformed_points[:,:3])
  return pcl_msg, transformed_points[:,:3]

def reset_arrays():
  global poses, poses_ts_s, poses_ts_ns, pcls, pcls_ts_s, pcls_ts_ns 
  poses = []
  poses_ts_s = []
  poses_ts_ns = []
  pcls = []
  pcls_ts_s = []
  pcls_ts_ns = []

def get_ts_match_idx(ts_s, ts_ns, target_s, target_ns):
  for i in range(len(ts_s)):
    if (ts_s[i] == target_s):
      if (ts_ns[i] == target_ns):
        return i
  return -1

c = 0
n = 100
open3d_pcls = []

bag = rosbag.Bag('test_rgbd.bag')
# out_bag = rosbag.Bag('out.bag', 'w')
# for topic, msg, t in bag.read_messages(topics=['chatter', 'numbers']):
for topic, msg, t in bag.read_messages():
  points = []
  for p in pcl2.read_points(msg, field_names = ("x", "y", "z", "r", "g", "b"), skip_nans=True):
    print("p: ", p)
    # points.append(np.asarray([p[0],p[1],p[2],p[3],p[4],p[5]]))
  print("Points: ", points)

# # Display the pointclouds using open3d
# print("Number of save pcls: ", len(open3d_pcls))
# pcds = []
# path = "./pcls/pcd"
# for i in range(0, n, 3):
#   print("i: ", i)
#   pcd = o3d.geometry.PointCloud()
#   pcd.points = o3d.utility.Vector3dVector(open3d_pcls[i])
#   o3d.io.write_point_cloud("%s_%s.ply" % (path, i), pcd)
#   # print(pcd)
#   # print(np.asarray(pcd.points))
#   # o3d.visualization.draw_geometries([pcd])
#   # pcds.append(pcd)

# # o3d.visualization.draw_geometries([pcds])


