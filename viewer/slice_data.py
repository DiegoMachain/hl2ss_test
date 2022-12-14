#!/usr/bin/env python3

import glob
import numpy as np
import open3d as o3d
import re
import argparse
import random
import csv

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

def import_pcds(files):
  pcds = []
  for file in files:
    pcd = o3d.io.read_point_cloud(file)
    pcds.append(pcd)
  return pcds

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

def get_min_max(pcd1, axis=1):
  min = 10000
  max = -10000
  for p in pcd1.points:
    if p[axis] < min:
      min = p[axis]
    if p[axis] > max:
      max = p[axis]
  return min, max

def get_free_points(point, trafo):
  pose = [trafo[0,3], trafo[1,3], trafo[2,3]]
  vector = point - pose 
  distance = np.sqrt(np.power(vector[0],2) + np.power(vector[1],2) + np.power(vector[2],2))
  num_points = np.maximum(1, (distance/2).astype(int))
  points = []
  labels = []
  
  for i in range(num_points):
    # draw random number uniformly between 0 and 1
    free_distance = distance * np.sqrt(random.uniform(0,1))
    free_distance = np.maximum(0.1, np.minimum(distance - 0.1, free_distance))
    free_point = [pose[0] + free_distance/distance * vector[0], pose[1] + free_distance/distance * vector[1], pose[2] + free_distance/distance * vector[2]]
    points.append(free_point)
    labels.append(0)

  return points, labels

# ----------------------------------------------------------------------------------------------------------- #

# python3 slice_data.py -f /home/toni/Documents/datasets/hololens_mike/output/run2_MAV_corridor/ -p world -he 1.3 -t 0.005

if __name__ == '__main__':
  argParser = argparse.ArgumentParser()
  argParser.add_argument("-f", "--folder", help="path to folder with ply files")
  argParser.add_argument("-p", "--pattern", help="part of file names to match to extract correct ones, e.g. world")
  argParser.add_argument("-he", "--height", help="Height at which to retrieve 2D data slice")
  argParser.add_argument("-t", "--tolerance", help="Tolerance band for slicing, e.g. 0.01 --> +- 1cm")

  args = argParser.parse_args()

  height = float(args.height)
  tolerance = float(args.tolerance)

  # Import pcd files 
  pcd_files = glob.glob("%s%s*.ply" % (args.folder, args.pattern))
  pcd_files.sort(key=natural_keys)
  pcds = import_pcds(pcd_files)

  # Import trafo files
  trafo_files = glob.glob("%strafo*" % args.folder)
  trafo_files.sort(key=natural_keys)
  trafos = import_trafos(trafo_files)

  # Get the min and max along the vertical axis 
  min = 10000
  max = -10000
  for i in range(len(pcd_files)):

    # read ply file
    pcd = o3d.io.read_point_cloud(pcd_files[i])

    # get the min and max of this pointcloud
    min_pcl, max_pcl = get_min_max(pcd)
    print("PCL min: %s, max: %s" % (min_pcl, max_pcl))
    if (min_pcl < min):
      min = min_pcl
    if (max_pcl > max):
      max = max_pcl

  # Print out the min and max value (vertical axis is -y, see Research api)
  print("Min vertical: %s, max vertical: %s" % (min, max))
  biased_height = height
  height_min = biased_height + tolerance
  height_max = biased_height - tolerance
  print("Slicing data at height %s with tolerance +-%s: %s to %s" % (biased_height, tolerance, height_min, height_max))

  # Open the output csv file 
  file = open('%sdataset.csv' % args.folder, 'w')
  writer = csv.writer(file)

  # Create new slice point clouds and save again
  sliced_pcls = []
  for i in range(len(pcd_files)):

    # read ply file
    pcd = o3d.io.read_point_cloud(pcd_files[i])

    # extract points at desired height
    points = []
    labels = []
    colors = []
    got_points = False
    for j in range(len(pcd.points)):
      p = pcd.points[j]
      if p[1] < height_min and p[1] > height_max:

        # Add occupied point 
        points.append(p)
        labels.append(1)                        # 1 = occupied
        color = np.ones((1,3))
        color[:,0] = 0.
        color[:,1] = 1.
        color[:,2] = 0.
        colors.append(np.squeeze(color))
        got_points = True

        # create free points 
        hololens_to_world = np.linalg.inv(trafos[i])
        free_points, free_labels = get_free_points(p, hololens_to_world)
        # print("Got free points: %s" % free_points)
        for i in range(len(free_points)):
          points.append(free_points[i])
          labels.append(0)
          color = np.ones((1,3))
          color[:,0] = 1.
          color[:,1] = 0.
          color[:,2] = 0.
          colors.append(np.squeeze(color))

    # write all points to csv file 
    # write out values in x-z-plane (vertical is -y)
    for j in range(len(points)):
      data = [str(points[j][0]), str(-points[j][2]), str(labels[j])]
      writer.writerow(data)

    print("Point cloud with %s points had %s points in the range" % (len(pcd.points), len(points)))

    # Now create a new point cloud
    pcd_new = o3d.geometry.PointCloud()
    pcd_new.points = o3d.utility.Vector3dVector(points)
    pcd_new.colors = o3d.utility.Vector3dVector(colors)

    # save transformed point cloud
    o3d.io.write_point_cloud("%sslicedpcd_%s.ply" % (args.folder, i), pcd_new)

    # # visualize the data --> only for debugging
    # if got_points:
    #   # create a line set from the position to the points 
    #   hololens_to_world = np.linalg.inv(trafos[i])
    #   # points.insert(0, [hololens_to_world[0,3], hololens_to_world[2,3], hololens_to_world[1,3]])
    #   points.insert(0, [hololens_to_world[0,3], hololens_to_world[1,3], hololens_to_world[2,3]])
    #   # print("Points 0: ", points[0])
    #   # print("Trafo: ", hololens_to_world)
    #   lines = []
    #   for j in range(1, len(points)-1):
    #     lines.append([0,j])
    #     # lines = [[0, 1], [0, 2], [0, 3], [0, 8], [0, 10]]
    #   colors = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    #   line_set = o3d.geometry.LineSet()
    #   line_set.points = o3d.utility.Vector3dVector(points)
    #   line_set.lines = o3d.utility.Vector2iVector(lines)
    #   line_set.colors = o3d.utility.Vector3dVector(colors)
    #   o3d.visualization.draw_geometries([line_set, pcd, pcd_new])

    #   # o3d.visualization.draw_geometries([pcd])
    #   got_points = False

  file.close()