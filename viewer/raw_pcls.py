#!/usr/bin/env python3

import glob
import numpy as np
import open3d as o3d
import re

# folder = "./pcls_final/"
# folder = "/home/toni/Documents/datasets/hololens_mike/output/run1_MAV_office/"
folder = "/home/toni/Documents/datasets/hololens_mike/output/run2_MAV_corridor/"

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
    # print("Line length: ", len(line))
    lines = line[0].split(";")
    print("Lines: ", lines)
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
  # points = pcl.points
  points = []
  for p in pcl.points:
    points.append(np.asarray([p[0],p[1],p[2],1]))

  # Generate transformation matrix 
  hololens_to_world = np.linalg.inv(world_to_hololens)

  # Transform each point in the pointcloud
  transformed_points = []
  c = 0
  for p in points:
    # if c == 0:
      # print("Point before: %s" % p)
    t_p = np.dot(hololens_to_world, p)
    # t_p[2] = t_p[2] * -1
    # if c == 0:
      # print("Point after: %s" % t_p)
    transformed_points.append(t_p)
    # c += 1

  # Replace the old points with new point cloud
  transformed_points = np.asarray(transformed_points)
  pcl.points = o3d.utility.Vector3dVector(transformed_points[:,:3])

  # transformed_points = np.asarray(transformed_points)
  return pcl


class Visualizer():
    def __init__(self):

        pcd_files = glob.glob("%sraw*.ply" % folder)
        pcd_files.sort(key=natural_keys)

        trafo_files = glob.glob("%strafo*" % folder)
        trafo_files.sort(key=natural_keys)

        # print("pcd_files 0: %s" % pcd_files)
        # print("pcd_files 0: %s" % pcd_files[0])
        # print("trafo_files 0: %s" % trafo_files[0])

        pcds = import_pcds(pcd_files)
        print(len(pcds))

        trafos = import_trafos(trafo_files)
        print(len(trafos))
        print("First trafo: ", trafos[0])
        print(trafos[0].shape)

        c = 0
        n = 5                                           # Number of point clouds to be displayed

        # read in all the files
        # for i, file in enumerate(pcd_files):
        # for i in range(len(pcd_files)-n):
        for i in range(5, 50):

            # if c < (len(files)-3):
            # if c < 10:

                # # read ply file
                # pcd1 = o3d.io.read_point_cloud(pcd_files[i])
                # pcd1_colors = np.ones((len(pcd1.points),3))
                # pcd1_colors[:,0] = 1.0
                # pcd1_colors[:,1] = 0.0
                # pcd1_colors[:,2] = 0.0
                # pcd1.colors = o3d.utility.Vector3dVector(pcd1_colors)

                # pcd2 = o3d.io.read_point_cloud(pcd_files[i+1])
                # pcd2_colors = np.ones((len(pcd2.points),3))
                # pcd2_colors[:,0] = 0.0
                # pcd2_colors[:,1] = 1.0
                # pcd2_colors[:,2] = 0.0
                # pcd2.colors = o3d.utility.Vector3dVector(pcd2_colors)

                # read ply file
                pcd1 = o3d.io.read_point_cloud(pcd_files[i])
                pcd2 = o3d.io.read_point_cloud(pcd_files[i+1])
                pcd3 = o3d.io.read_point_cloud(pcd_files[i+2])
                pcd4 = o3d.io.read_point_cloud(pcd_files[i+3])
                pcd5 = o3d.io.read_point_cloud(pcd_files[i+4])


                # pcd3 = o3d.io.read_point_cloud(pcd_files[i+2])
                print("i: %s" % i)
                print("Number of points: %s" % len(pcd1.points))
                # print("Trafo %s: %s" % (i, trafos[i]))
                # print("Trafo %s: %s" % (i+1, trafos[i+1]))

                # # visualize and save a couple of screenshots
                o3d.visualization.draw_geometries([pcd1, pcd2, pcd3, pcd4, pcd5])
                # o3d.visualization.draw_geometries([pcd1, pcd2, pcd3])

                # Now transform the data
                pcd1 = transform_data(pcd1, trafos[i])
                pcd2 = transform_data(pcd2, trafos[i+1])
                pcd3 = transform_data(pcd3, trafos[i+2])
                pcd4 = transform_data(pcd4, trafos[i+3])
                pcd5 = transform_data(pcd5, trafos[i+4])

                o3d.visualization.draw_geometries([pcd1, pcd2, pcd3, pcd4, pcd5], "After transform")

                c += 1
    
        #     # output path
        #     parts = file.split(".")
        #     parts = parts[0].split("mesh")
        #     # num = int(parts[-1])
        #     path = folder + "mesh" + parts[-1] + ".png"

        #     # save screenshots
        #     vis = o3d.visualization.Visualizer()
        #     vis.create_window()
        #     vis.add_geometry(pcd)
        #     ctr = vis.get_view_control()
        #     print("Field of view (before changing) %.2f" % ctr.get_field_of_view())
        #     ctr.change_field_of_view(step=30)
        #     print("Field of view (after changing) %.2f" % ctr.get_field_of_view())
        # # vis.run()
        # # vis.destroy_window()
        #     vis.update_geometry(pcd)
        #     vis.poll_events()
        #     vis.update_renderer()
        #     vis.capture_screen_image(path)
        #     vis.destroy_window()

if __name__ == '__main__':
    Visualizer()
