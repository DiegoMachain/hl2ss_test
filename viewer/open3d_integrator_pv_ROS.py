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
import crop_pcl
import numpy as np
import sys
import time

from std_msgs.msg import Float64MultiArray
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from robotics_demo. msg import PosRot, State
from std_msgs.msg import Header, String


#Function to draw the reference frame
def getCoordinateSystem(size):
    points = [[0, 0, 0], [size, 0, 0], [0, size, 0], [0, 0, size]]
    lines = [[0, 1], [0, 2], [0, 3]]
    colors = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(colors)
    return line_set

# Settings --------------------------------------------------------------------
#Initiate the nodes for ROS communication


class controler():
    def __init__(self):
        self.first_pcd = True
        self.bbox = False
        self.pcd = False
        self.vis = False
        self.cont = 0
        self.srcCont = 0 #Before interaction
        self.dstCont = 0 #After 
        self.state = None
        self.stopBbox = False
        self.numPCD = 10 #Number of point clouds to send to Ditto
        self.sendData = False #Flag to see if we send data to ditto or we wait
        self.QRPose = Float64MultiArray() #Pose to send to pcd_listener
        print("Init")
        
    def call_state(self, data):

        self.state = data


control = controler()


# HoloLens address
# host = "192.168.1.7"
host = "10.10.10.218"

calibration_path = 'output/calibration'

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

enable = True

def on_press(key):
    global enable
    enable = key != keyboard.Key.space
    return enable

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
# focal length x, focal length y, center x, center y
intrinsics_depth = o3d.camera.PinholeCameraIntrinsic(hl2ss.Parameters_RM_DEPTH_LONGTHROW.WIDTH, hl2ss.Parameters_RM_DEPTH_LONGTHROW.HEIGHT, calibration_lt.intrinsics[0, 0], calibration_lt.intrinsics[1, 1], calibration_lt.intrinsics[2, 0], calibration_lt.intrinsics[2, 1])

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

count = 0

line_set = getCoordinateSystem(1)




#Listen to the pose
def callback(data):
    sink_depth.acquire()

    data_lt = sink_depth.get_most_recent_frame()

    _, data_pv = sink_pv.get_nearest(data_lt.timestamp)

    if (data_pv is not None):
        if (hl2ss.is_valid_pose(data_pv.pose)):
        

            depth = hl2ss_3dcv.rm_depth_normalize(data_lt.payload.depth, calibration_lt.undistort_map, scale)
            depth_to_pv_image = hl2ss_3dcv.camera_to_rignode(calibration_lt.extrinsics) @ hl2ss_3dcv.reference_to_world(data_lt.pose) @ hl2ss_3dcv.world_to_reference(data_pv.pose) @ calibration_pv.intrinsics

            rgb, depth = hl2ss_3dcv.rm_depth_rgbd_registered(depth, data_pv.payload, xy1, depth_to_pv_image, cv2.INTER_LINEAR)
            rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(o3d.geometry.Image(rgb), o3d.geometry.Image(depth), depth_scale=1, depth_trunc=max_depth, convert_rgb_to_intensity=False)
            depth_world_to_camera = hl2ss_3dcv.world_to_reference(data_lt.pose) @ hl2ss_3dcv.rignode_to_camera(calibration_lt.extrinsics)

            #Camera to world
            volume.integrate(rgbd, intrinsics_depth, (depth_world_to_camera.transpose()))
            pcd_tmp = volume.extract_point_cloud()


            #Add another button to start bbox, decouple bbox and QR scanning
            #Start scanning then couple of the main
        
            #Publish the point clouds
            #It saves ten point clouds

            if control.state != None:
                # control == 5 is equal to starting the Bbox scanning

                print("State == ", control.state.state)
                if control.state.state == 5.:
                    
                    #sleep for a second
                    time.sleep(1)
                    #Create the bounding box
                    control.first_pcd = False
                    control.pcd = pcd_tmp
                    
                    #Flag to create bounding box
                    if not control.bbox:
                        points = crop_pcl.pick_points(control.pcd)
                        picked_points = np.asarray(control.pcd.points)[points]
                        picked_points = o3d.utility.Vector3dVector(picked_points)
                        control.bbox = o3d.geometry.OrientedBoundingBox.create_from_points(picked_points)
                        #bbox = o3d.geometry.AxisAlignedBoundingBox.create_from_points(picked_points)

                    print("Bounding box = ", control.bbox)
                    control.pcd = control.pcd.crop(control.bbox)

                    #Transformation matrix 
                
                    if data.pos_x != 0.:
                        T = np.eye(4)
                        T[:3, :3] = control.pcd.get_rotation_matrix_from_xyz((0, 0, 0))
                        T[0,3] = -data.pos_x
                        T[1,3] = -data.pos_y
                        T[2,3] = -data.pos_z

                        #Translate the mesh with the T matrix
                        
                        #No transformation
                        #control.pcd = control.pcd.transform(T)


                    control.vis = o3d.visualization.Visualizer()
                    control.vis.create_window()
                    control.vis.add_geometry(control.pcd)
                    control.vis.add_geometry(line_set)

                #Stop the scanning to obtain the bounding box
                elif control.state.state == 6.:
                    print("Stop bbox scanning")
                    control.stopBbox = True


                #control == 0 Start the scanning for the
                elif control.state.state == 0. and control.srcCont < control.numPCD:
                    control.srcCont += 1
                    header.frame_id = "before"
                    control.sendData = True
                    print("---Start first interaction---")

                # control == 1 stop the first interaction
                elif control.state.state == 1. and control.srcCont < control.numPCD:
                    #This enters just once
                    print("---Stop first interaction---")
                    control.sendData == False
                

                # control == 2 is equal to starting the the scann for after interaction
                elif control.state.state == 2. and control.dstCont < control.numPCD:
                    control.dstCont += 1
                    header.frame_id = "after"
                    control.sendData = True
                    print("---Start second interaction---")

                # control == 3 stopping the scan after the interaction
                elif control.state.state == 3. and control.dstCont < control.numPCD:
                    print("---Stop second interaction---")
                    control.dstCont == control.numPCD
                    control.sendData = False
                
                # control == 2 end of the process and start Ditto
                elif control.state.state == 4.:

                    print("--Send data to Ditto---")
                    header.frame_id = "end"

                    control.sendData = True
                    control.srcCont = 0
                    control.dstCont = 0
                    control.state = None
                #control == 7 stop all the process and don't send data
                elif control.state.state == 7.:
                    print("--Stop all process---")
                    control.sendData = False

                #don't publish until the bounding box has been created
                if control.stopBbox and control.sendData:
                    #
                    #crop pcd_tmp
                    pcd_tmp = pcd_tmp.crop(control.bbox)

                    control.pcd.points = pcd_tmp.points
                    control.pcd.colors = pcd_tmp.colors
                    
                    if data.pos_x != 0.:
                        T = np.eye(4)
                        T[:3, :3] = control.pcd.get_rotation_matrix_from_xyz((0, 0, 0))
                        T[0,3] = -data.pos_x
                        T[1,3] = -data.pos_y
                        T[2,3] = -data.pos_z

                        #Translate the mesh with the T matrix
                        #control.pcd = control.pcd.transform(T)

                        #Publish the QR to pcd_listener
                        
                        control.QRPose.data = [[data.pos_x, data.pos_y, data.pos_z, data.rot_x, data.rot_y, data.rot_z]]
                        
                        pubQR.publish(control.QRPose)



                    #pcd = pcd.crop(bbox)
                    control.vis.update_geometry(control.pcd)

                    #Publish the point cloud
                    pc2 = point_cloud2.create_cloud(header, fields, np.asarray(control.pcd.points))
                    pc2.header.stamp = rospy.Time.now()
                    pub.publish(pc2)  
                control.vis.poll_events()
                control.vis.update_renderer()          

            #Prepare the sending to Ditto

# Settings --------------------------------------------------------------------
#Initiate the nodes for ROS communication


rospy.init_node('listener', anonymous=True)
rospy.Subscriber("pos_rot", PosRot, callback) #pose of the QR code
rospy.Subscriber("state", State, control.call_state) 

#We want to use control.pcd to send to ditto
pub = rospy.Publisher("point_cloud2", PointCloud2, queue_size=2)
fields = [PointField('x', 0, PointField.FLOAT32, 1),
          PointField('y', 4, PointField.FLOAT32, 1),
          PointField('z', 8, PointField.FLOAT32, 1),
          PointField('rgb', 12, PointField.UINT32, 0),
          PointField('state', 16, PointField.UINT32,0),
          #PointField('rgba', 12, PointField.UINT32, 1),
          ]
header = Header()

#Get the state of the HoloLens buttons
pubQR = rospy.Publisher("QR", Float64MultiArray, queue_size = 10)



rospy.spin()









bbox = False

# HoloLens address
# host = "192.168.1.7"
host = "10.10.10.218"

calibration_path = 'output/calibration'

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

#------------------------------------------------------------------------------

#Function to draw the reference frame
def getCoordinateSystem(size):
    points = [[0, 0, 0], [size, 0, 0], [0, size, 0], [0, 0, size]]
    lines = [[0, 1], [0, 2], [0, 3]]
    colors = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(colors)
    return line_set

line_set = getCoordinateSystem(1)

#Receive the pose from the

if __name__ == '__main__':
    enable = True

    def on_press(key):
        global enable
        enable = key != keyboard.Key.space
        return enable

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
    # focal length x, focal length y, center x, center y
    intrinsics_depth = o3d.camera.PinholeCameraIntrinsic(hl2ss.Parameters_RM_DEPTH_LONGTHROW.WIDTH, hl2ss.Parameters_RM_DEPTH_LONGTHROW.HEIGHT, calibration_lt.intrinsics[0, 0], calibration_lt.intrinsics[1, 1], calibration_lt.intrinsics[2, 0], calibration_lt.intrinsics[2, 1])

    
    
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
 
    count = 0

    while (enable):
        sink_depth.acquire()

        data_lt = sink_depth.get_most_recent_frame()
        if (not hl2ss.is_valid_pose(data_lt.pose)):
            continue

        _, data_pv = sink_pv.get_nearest(data_lt.timestamp)
        if ((data_pv is None) or (not hl2ss.is_valid_pose(data_pv.pose))):
            continue

        depth = hl2ss_3dcv.rm_depth_normalize(data_lt.payload.depth, calibration_lt.undistort_map, scale)
        depth_to_pv_image = hl2ss_3dcv.camera_to_rignode(calibration_lt.extrinsics) @ hl2ss_3dcv.reference_to_world(data_lt.pose) @ hl2ss_3dcv.world_to_reference(data_pv.pose) @ calibration_pv.intrinsics

        rgb, depth = hl2ss_3dcv.rm_depth_rgbd_registered(depth, data_pv.payload, xy1, depth_to_pv_image, cv2.INTER_LINEAR)
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(o3d.geometry.Image(rgb), o3d.geometry.Image(depth), depth_scale=1, depth_trunc=max_depth, convert_rgb_to_intensity=False)
        depth_world_to_camera = hl2ss_3dcv.world_to_reference(data_lt.pose) @ hl2ss_3dcv.rignode_to_camera(calibration_lt.extrinsics)

        volume.integrate(rgbd, intrinsics_depth, depth_world_to_camera.transpose())
        pcd_tmp = volume.extract_point_cloud()


        if (first_pcd):
            first_pcd = False
            pcd = pcd_tmp
            
            #Flag to create bounding box
            if not bbox:
                points = crop_pcl.pick_points(pcd)
                picked_points = np.asarray(pcd.points)[points]
                picked_points = o3d.utility.Vector3dVector(picked_points)
                bbox = o3d.geometry.OrientedBoundingBox.create_from_points(picked_points)
                #bbox = o3d.geometry.AxisAlignedBoundingBox.create_from_points(picked_points)

            pcd = pcd.crop(bbox)

            vis = o3d.visualization.Visualizer()
            vis.create_window()
            vis.add_geometry(pcd)
            #vis.add_geometry(line_set)
        else:

            #crop pcd_tmp
            pcd_tmp = pcd_tmp.crop(bbox)
            pcd.points = pcd_tmp.points
            pcd.colors = pcd_tmp.colors

            #pcd = pcd.crop(bbox)

            vis.update_geometry(pcd)


        vis.poll_events()
        vis.update_renderer()

        #Integrator needs to send the images to the listener


    [sink.detach() for sink in sinks]
    producer.stop(hl2ss.StreamPort.PERSONAL_VIDEO)
    producer.stop(hl2ss.StreamPort.RM_DEPTH_LONGTHROW)
    listener.join()

    vis.run()

    hl2ss.stop_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)
