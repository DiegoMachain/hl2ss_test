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
  # world_to_hololens = tl.fromTranslationRotation(translation, rotation)
  # hololens_to_world = np.linalg.inv(world_to_hololens)
  hololens_to_world = tl.fromTranslationRotation(translation, rotation)
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

bag = rosbag.Bag('test_translation.bag')
out_bag = rosbag.Bag('out.bag', 'w')
# for topic, msg, t in bag.read_messages(topics=['chatter', 'numbers']):
for topic, msg, t in bag.read_messages():
  # save the msg data 
  # print(topic)
  # print(msg.header.stamp)
  # print(msg.header.stamp.nsecs)
  # print(msg.header.stamp.secs)
  if topic == "hololens_pose":
    # check if there are pcl messages:
    if len(pcls) > 0:
      # check if there is matching timestep
      match = get_ts_match_idx(pcls_ts_s, pcls_ts_ns, msg.header.stamp.secs, msg.header.stamp.nsecs)
      if (match != -1):
          # print("Found match in pcls time stamps at %s" % match)
          # print("Target: s: %s, ns: %s, Match: s: %s, ns: %s" % (msg.header.stamp.secs, msg.header.stamp.nsecs, pcls_ts_s[match], pcls_ts_ns[match]))
          transformed_pcl, points = transform_data(msg, pcls[match])
          out_bag.write('hololens_pcl', transformed_pcl)
          if (c < n):
            open3d_pcls.append(points)
            c += 1
          reset_arrays()
      else:
        # print("Appending pose msg")    
        # save pose messages 
        poses.append(msg)
        poses_ts_s.append(msg.header.stamp.secs)
        poses_ts_ns.append(msg.header.stamp.nsecs)
    else: 
      # print("Appending pose msg")    
      # save pose messages 
      poses.append(msg)
      poses_ts_s.append(msg.header.stamp.secs)
      poses_ts_ns.append(msg.header.stamp.nsecs)
  if topic == "hololens_pcl":
    # check if there are pose messages:
    if len(poses) > 0:
      # check if the timestamps are the same 
      match = get_ts_match_idx(poses_ts_s, poses_ts_ns, msg.header.stamp.secs, msg.header.stamp.nsecs)
      if (match != -1):
        # print("Found match in poses time stamps at %s" % match)
        # print("Target: s: %s, ns: %s, Match: s: %s, ns: %s" % (msg.header.stamp.secs, msg.header.stamp.nsecs, pcls_ts_s[match], pcls_ts_ns[match]))
        transformed_pcl, points = transform_data(poses[match], msg)
        out_bag.write('hololens_pcl', transformed_pcl)
        if (c < n):
          open3d_pcls.append(points)
          c += 1
        reset_arrays()
      else:
        # print("Appending pcl msg") 
        # save pcl messages 
        pcls.append(msg)
        pcls_ts_s.append(msg.header.stamp.secs)
        pcls_ts_ns.append(msg.header.stamp.nsecs)
    else:
      # print("Appending pcl msg") 
      # save pcl messages 
      pcls.append(msg)
      pcls_ts_s.append(msg.header.stamp.secs)
      pcls_ts_ns.append(msg.header.stamp.nsecs)
bag.close()

# Display the pointclouds using open3d
print("Number of save pcls: ", len(open3d_pcls))
pcds = []
path = "./pcls/pcd"
# for i in range(0, n, 3):
for i in range(48, n, 3):

  # pcd.colors = Vector3dVector(np_colors)
  print("i: ", i)
  pcd1_colors = np.ones(open3d_pcls[i].shape)
  pcd1_colors[:,0] = 1.0
  pcd1_colors[:,1] = 0.0
  pcd1_colors[:,2] = 0.0
  pcd1 = o3d.geometry.PointCloud()
  pcd1.points = o3d.utility.Vector3dVector(open3d_pcls[i])
  pcd1.colors = o3d.utility.Vector3dVector(pcd1_colors)
  pcd2_colors = np.ones(open3d_pcls[i+3].shape)
  pcd2_colors[:,0] = 0.0
  pcd2_colors[:,1] = 1.0
  pcd2_colors[:,2] = 0.0
  pcd2 = o3d.geometry.PointCloud()
  pcd2.points = o3d.utility.Vector3dVector(open3d_pcls[i+3])
  pcd2.colors = o3d.utility.Vector3dVector(pcd2_colors)
  o3d.visualization.draw_geometries([pcd1, pcd2])
  # o3d.io.write_point_cloud("%s_%s.ply" % (path, i), pcd)
  # print(pcd)
  # print(np.asarray(pcd.points))
  # o3d.visualization.draw_geometries([pcd])
  # pcds.append(pcd)

# o3d.visualization.draw_geometries([pcds])


