#------------------------------------------------------------------------------
# This script receives encoded video from the HoloLens depth camera in long 
# throw mode and plays it. The resolution is 320x288 @ 5 FPS. The stream
# supports three operating modes: 0) video, 1) video + rig pose, 2) query
# calibration (single transfer). Press esc to stop. Depth and AB data are 
# scaled for visibility. Note that the ahat and long throw streams cannot be
# used simultaneously.
#------------------------------------------------------------------------------

from pynput import keyboard

import open3d as o3d
import numpy as np
import hl2ss_3dcv
import hl2ss
import cv2

import rospy
import rosbag
from std_msgs.msg import Header
from sensor_msgs.msg import PointField
import sensor_msgs.point_cloud2 as pcl2
from geometry_msgs.msg import PoseStamped
import tf

# Settings --------------------------------------------------------------------

# HoloLens address
# host = "192.168.1.7"
host = "10.10.10.218"

# Port
port = hl2ss.StreamPort.RM_DEPTH_LONGTHROW

# Operating mode
# 0: video
# 1: video + rig pose
# 2: query calibration (single transfer)
mode = hl2ss.StreamMode.MODE_1

# PNG filter
png_filter = hl2ss.PngFilterMode.Paeth

# Max depth in meters
max_depth = 10.0

# Output path 

#------------------------------------------------------------------------------

if (mode == hl2ss.StreamMode.MODE_2):
    data = hl2ss.download_calibration_rm_depth_longthrow(host, port)
    print('Calibration data')
    print(data.uv2xy.shape)
    print(data.extrinsics)
    print(data.scale)
    print(data.undistort_map.shape)
    print(data.intrinsics)
    quit()

enable = True

def on_press(key):
    global enable
    enable = key != keyboard.Key.esc
    return enable

listener = keyboard.Listener(on_press=on_press)
listener.start()

# Get camera calibration

calibration = hl2ss.download_calibration_rm_depth_longthrow(host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW)

# Compute depth-to-rgb registration constants

xy1, scale, _ = hl2ss_3dcv.rm_depth_registration(calibration.uv2xy, calibration.scale, calibration.extrinsics, calibration.extrinsics, calibration.extrinsics)

# Init ROS
rospy.init_node('HoloLensLogger', anonymous=False)

# Open rosbag

bag = rosbag.Bag('test_translation.bag', 'w')

client = hl2ss.rx_decoded_rm_depth_longthrow(host, port, hl2ss.ChunkSize.RM_DEPTH_LONGTHROW, mode, png_filter)
client.open()

while (enable):
    data = client.get_next_packet()
    
    # Convert depth to 3D points
    depth = hl2ss_3dcv.rm_depth_scale(data.payload.depth, scale)

    xyz = hl2ss_3dcv.rm_depth_to_points(depth, xy1)
    xyz = xyz[(xyz[:, 2] > 0) & (xyz[:, 2] < max_depth), :] 

    # pcd = o3d.geometry.PointCloud()
    # pcd.points = o3d.utility.Vector3dVector(xyz)

    # o3d.visualization.draw_geometries([pcd])

    # Generate ROS PCL message 
    header = Header()
    t = rospy.Time.now()
    print("t: ", t)
    header.stamp = t
    print("pcl stamp: ", header.stamp)
    header.frame_id = "/hololens"
    # Points have to be filled exactly like this to enable conversion via pcl
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1)]
    pcl_msg = pcl2.create_cloud(header, fields, xyz)

    # Generate ROS Pose message 
    # cur_matrix = matrix.reshape(3,4)
    # cur_matrix_homo = np.vstack((cur_matrix, np.array([0, 0, 0, 1]))) # to homogenous coordinates
    print('Pose at time {ts}'.format(ts=data.timestamp))
    t_pose = data.pose.transpose()
    print(data.pose.transpose())
    q = tf.transformations.quaternion_from_matrix(t_pose)
    pose_msg = PoseStamped()
    header_pose = Header()
    header_pose.stamp = t
    print("pose stamp: ", header_pose.stamp)
    header_pose.frame_id = "/hololens"
    pose_msg.header = header_pose
    pose_msg.header.stamp = t
    pose_msg.pose.position.x = t_pose[0][3]
    pose_msg.pose.position.y = t_pose[1][3]
    pose_msg.pose.position.z = t_pose[2][3]
    pose_msg.pose.orientation.x = q[0]
    pose_msg.pose.orientation.y = q[1]
    pose_msg.pose.orientation.z = q[2]
    pose_msg.pose.orientation.w = q[3]

    # Save messages rosbag
    bag.write('hololens_pcl', pcl_msg)
    bag.write('hololens_pose', pose_msg)

    # print('Pose at time {ts}'.format(ts=data.timestamp))
    # print(data.pose)
    # cv2.imshow('Depth', data.payload.depth / np.max(data.payload.depth)) # Normalized for visibility
    # cv2.imshow('AB', data.payload.ab / np.max(data.payload.ab)) # Normalized for visibility
    # cv2.waitKey(1)

client.close()
listener.join()
