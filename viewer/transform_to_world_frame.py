#!/usr/bin/env python3

import glob
import numpy as np
import open3d as o3d
import re
import argparse

# folder = "/home/toni/Documents/datasets/hololens_mike/output/run1_MAV_office/"
# folder = "/home/toni/Documents/datasets/hololens_mike/output/run2_MAV_corridor/"

# acc. to https://stackoverflow.com/questions/5967500/how-to-correctly-sort-a-string-with-a-number-inside
def atoi(text):
    return int(text) if text.isdigit() else text

def natural_keys(text):
    '''
    alist.sort(key=natural_keys) sorts in human order
    http://nedbatchelder.com/blog/200712/human_sorting.html
    (See Toothy's implementation in the comments)
    '''
    return [ atoi(c) for c in re.split(r'(\d+)', text) ]

def import_trafos(paths):
  trafos = []
  for path in paths:
    # Open file
    file = open(path, "r+")
    # read lines
    line = file.readlines()
    lines = line[0].split(";")
    list = []
    for line in lines[:-1]:
        vals = line.split(",")
        list.append([float(i) for i in vals])
    # close file
    file.close()
    trafos.append(np.asarray(list))
  return np.asarray(trafos)

def import_pcds(files):
  pcds = []
  for file in files:
    pcd = o3d.io.read_point_cloud(file)
    pcds.append(pcd)
  return pcds

def transform_data(pcl, world_to_hololens):
  # Points to be transformed
  points = []
  for p in pcl.points:
    points.append(np.asarray([p[0],p[1],p[2],1]))

  # Generate transformation matrix 
  hololens_to_world = np.linalg.inv(world_to_hololens)

  # Transform each point in the pointcloud
  transformed_points = []
  c = 0
  for p in points:
    t_p = np.dot(hololens_to_world, p)
    transformed_points.append(t_p)

  # Replace the old points with new point cloud
  transformed_points = np.asarray(transformed_points)
  pcl.points = o3d.utility.Vector3dVector(transformed_points[:,:3])

  return pcl

# ----------------------------------------------------------------------------------------------------------- #

if __name__ == '__main__':
  argParser = argparse.ArgumentParser()
  argParser.add_argument("-f", "--folder", help="path to folder with ply files and trafos")

  args = argParser.parse_args()

  # print("args.name=%s" % args.folder)

  # Import pcd files 
  pcd_files = glob.glob("%sraw*.ply" % args.folder)
  pcd_files.sort(key=natural_keys)
  pcds = import_pcds(pcd_files)

  # Import trafo files
  trafo_files = glob.glob("%strafo*" % args.folder)
  trafo_files.sort(key=natural_keys)
  trafos = import_trafos(trafo_files)

  # Transform the point clouds to the world frame and save again
  for i in range(len(pcd_files)):

    # read ply file
    pcd1 = o3d.io.read_point_cloud(pcd_files[i])

    # Now transform the data
    pcd1 = transform_data(pcd1, trafos[i])

    # save transformed point cloud
    o3d.io.write_point_cloud("%sworldpcd_%s.ply" % (args.folder, i), pcd1)
