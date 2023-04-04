#!/usr/bin/env python
#------------------------------------------------------------------------------
# RGBD integration using Open3D. Color information comes from the front RGB
# camera. Press space to stop.
#------------------------------------------------------------------------------
# TODO: USE CAMERA POSES FOR MORE ACCURATE REGISTRATION

from pynput import keyboard

import multiprocessing as mp
import open3d as o3d
import cv2
import hl2ss
import hl2ss_mp
import hl2ss_3dcv

import rospy
from std_msgs.msg import String

import numpy as np

# import rospy
# import rosbag
# from std_msgs.msg import Header
# from sensor_msgs.msg import PointField
# import sensor_msgs.point_cloud2 as pcl2
# from geometry_msgs.msg import PoseStamped
# import tf


#Function for the ros pose information

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('chatter', String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

listener()

# Settings --------------------------------------------------------------------

# HoloLens address
# host = "192.168.1.7"
host = "10.10.10.218"

calibration_path = './calibration'

# Camera parameters
focus = 1000
width = 640
height = 360
framerate = 30
profile = hl2ss.VideoProfile.H265_MAIN
bitrate = 5*1024*1024
exposure_mode = hl2ss.PV_ExposureMode.Manual
exposure = hl2ss.PV_ExposureValue.Max // 4
iso_speed_mode = hl2ss.PV_IsoSpeedMode.Manual
iso_speed_value = 1600
white_balance = hl2ss.PV_ColorTemperaturePreset.Manual

# Buffer length in seconds
buffer_length = 10

# Integration parameters
voxel_length = 1/100
sdf_trunc = 0.04
max_depth = 3.0

def writeData(path, data):
    file = open(path, "w")
    trafo = np.asarray(data)
    for i in range(np.shape(data)[0]):
        line = "%s, %s, %s, %s;" % (data[i][0], data[i][1], data[i][2], data[i][3])
        file.write(line)
    # close file
    file.close()

#------------------------------------------------------------------------------

if __name__ == '__main__':
    enable = True

    def on_press(key):
        global enable
        enable = key != keyboard.Key.space
        return enable

    # # Init ROS
    # rospy.init_node('HoloLensLogger', anonymous=False)

    # # Open rosbag

    # bag = rosbag.Bag('test_rgbd.bag', 'w')

    # Change this path for multiple runs
    path = "./output/run1/"
    count = 0

    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    hl2ss.start_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)
    rc_control = hl2ss.tx_rc(host, hl2ss.IPCPort.REMOTE_CONFIGURATION)
    while (not rc_control.get_pv_subsystem_status()):
        pass

    hl2ss_3dcv.pv_optimize_for_cv(host, focus, exposure_mode, exposure, iso_speed_mode, iso_speed_value, white_balance)

    calibration_pv = hl2ss_3dcv.get_calibration_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO, calibration_path, focus, width, height, framerate, True)
    calibration_lt = hl2ss.download_calibration_rm_depth_longthrow(host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW)
    # calibration_lt = hl2ss_3dcv.get_calibration_rm(host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW, calibration_path)

    uv2xy = hl2ss_3dcv.compute_uv2xy(calibration_lt.intrinsics, hl2ss.Parameters_RM_DEPTH_LONGTHROW.WIDTH, hl2ss.Parameters_RM_DEPTH_LONGTHROW.HEIGHT)
    xy1, scale, depth_to_pv_image = hl2ss_3dcv.rm_depth_registration(uv2xy, calibration_lt.scale, calibration_lt.extrinsics, calibration_pv.intrinsics, calibration_pv.extrinsics)

    volume = o3d.pipelines.integration.ScalableTSDFVolume(voxel_length=voxel_length, sdf_trunc=sdf_trunc, color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8)
    intrinsics_depth = o3d.camera.PinholeCameraIntrinsic(hl2ss.Parameters_RM_DEPTH_LONGTHROW.WIDTH, hl2ss.Parameters_RM_DEPTH_LONGTHROW.HEIGHT, calibration_lt.intrinsics[0, 0], calibration_lt.intrinsics[1, 1], calibration_lt.intrinsics[2, 0], calibration_lt.intrinsics[2, 1])

    vis = o3d.visualization.Visualizer()
    vis.create_window()
    first_pcd = True

    producer = hl2ss_mp.producer()
    producer.configure_pv(True, host, hl2ss.StreamPort.PERSONAL_VIDEO, hl2ss.ChunkSize.PERSONAL_VIDEO, hl2ss.StreamMode.MODE_1, width, height, framerate, profile, bitrate, 'rgb24')
    producer.configure_rm_depth_longthrow(True, host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW, hl2ss.ChunkSize.RM_DEPTH_LONGTHROW, hl2ss.StreamMode.MODE_1, hl2ss.PngFilterMode.Paeth)
    producer.initialize(hl2ss.StreamPort.PERSONAL_VIDEO, framerate * buffer_length)
    producer.initialize(hl2ss.StreamPort.RM_DEPTH_LONGTHROW, hl2ss.Parameters_RM_DEPTH_LONGTHROW.FPS * buffer_length)
    producer.start(hl2ss.StreamPort.PERSONAL_VIDEO)
    producer.start(hl2ss.StreamPort.RM_DEPTH_LONGTHROW)

    manager = mp.Manager()
    consumer = hl2ss_mp.consumer()
    sink_pv = consumer.create_sink(producer, hl2ss.StreamPort.PERSONAL_VIDEO, manager, None)
    sink_depth = consumer.create_sink(producer, hl2ss.StreamPort.RM_DEPTH_LONGTHROW, manager, ...)

    sinks = [sink_pv, sink_depth]

    [sink.get_attach_response() for sink in sinks]
 
    while (enable):
        sink_depth.acquire()

        data_lt = sink_depth.get_most_recent_frame()
        if (not hl2ss.is_valid_pose(data_lt.pose)):
            continue

        _, data_pv = sink_pv.get_nearest(data_lt.timestamp)
        if ((data_pv is None) or (not hl2ss.is_valid_pose(data_pv.pose))):
            continue

        depth = hl2ss_3dcv.rm_depth_normalize(data_lt.payload.depth, calibration_lt.undistort_map, scale)
        # pose of pv frame directly gives pv to world as opposed to RM sensors 
        depth_to_pv_image = hl2ss_3dcv.camera_to_rignode(calibration_lt.extrinsics) @ hl2ss_3dcv.reference_to_world(data_lt.pose) @ hl2ss_3dcv.world_to_reference(data_pv.pose) @ calibration_pv.intrinsics

        rgb, depth = hl2ss_3dcv.rm_depth_rgbd_registered(depth, data_pv.payload, xy1, depth_to_pv_image, cv2.INTER_LINEAR)
        # print("rgb: ", rgb)
        # print("depth: ", depth)
        # print("intrinsics_depth", intrinsics_depth.intrinsic_matrix)
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(o3d.geometry.Image(rgb), o3d.geometry.Image(depth), depth_scale=1, depth_trunc=max_depth, convert_rgb_to_intensity=False)

        pcd_tmp = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsics_depth)
        # gives the transformation from world to depth_camera
        depth_world_to_camera = hl2ss_3dcv.world_to_reference(data_lt.pose) @ hl2ss_3dcv.rignode_to_camera(calibration_lt.extrinsics)

        o3d.io.write_point_cloud("%sraw_pcd_%s.ply" % (path, count), pcd_tmp)

        # Flip it, otherwise the pointcloud will be upside down
        pcd_tmp.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        # print(depth_world_to_camera.transpose())
        # pcd_tmp.transform(depth_world_to_camera.transpose())
        # o3d.visualization.draw_geometries([pcd], zoom=0.5)

        # print("Before saving")
        o3d.io.write_point_cloud("%spcd_%s.ply" % (path, count), pcd_tmp)
        count = count + 1
        trafo_path = "%strafo_%s" % (path, count)
        writeData(trafo_path, depth_world_to_camera.transpose())
        intr_path = "%sintr_%s" % (path, count)
        writeData(intr_path, intrinsics_depth.intrinsic_matrix)
        # print("After saving")

        #  # Generate ROS PCL message 
        # header = Header()
        # t = rospy.Time.now()
        # print("t: ", t)
        # header.stamp = t
        # print("pcl stamp: ", header.stamp)
        # header.frame_id = "/hololens"
        # # Points have to be filled exactly like this to enable conversion via pcl
        # fields = [PointField('x', 0, PointField.FLOAT32, 1),
        #             PointField('y', 4, PointField.FLOAT32, 1),
        #             PointField('z', 8, PointField.FLOAT32, 1), ]
        # pcl_msg = pcl2.create_cloud(header, fields, pcd_tmp.points)

        # # Save messages rosbag
        # bag.write('hololens_pcl', pcl_msg)

        volume.integrate(rgbd, intrinsics_depth, depth_world_to_camera.transpose())
        pcd_tmp = volume.extract_point_cloud()

        if (first_pcd):
            first_pcd = False
            pcd = pcd_tmp
            vis.add_geometry(pcd)
        else:
            pcd.points = pcd_tmp.points
            pcd.colors = pcd_tmp.colors
            vis.update_geometry(pcd)

        vis.poll_events()
        vis.update_renderer()

    [sink.detach() for sink in sinks]
    producer.stop(hl2ss.StreamPort.PERSONAL_VIDEO)
    producer.stop(hl2ss.StreamPort.RM_DEPTH_LONGTHROW)
    listener.join()

    vis.run()

    hl2ss.stop_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)
