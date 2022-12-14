#!/usr/bin/env python3

import glob
import numpy as np
import open3d as o3d
import argparse
import csv
import sys
import pandas as pd
import matplotlib.pyplot as plt

# ----------------------------------------------------------------------------------------------------------- #

# python3 slice_data.py -f /home/toni/Documents/datasets/hololens_mike/output/run2_MAV_corridor/ -p world -he 1.3 -t 0.005

if __name__ == '__main__':
  argParser = argparse.ArgumentParser()
  argParser.add_argument("-f", "--file", help="path to csv file")

  args = argParser.parse_args()

  free_points = []
  hit_points = []  

  try:
      with open(args.file) as csvfile:
          reader = csv.reader(csvfile, delimiter=' ', quotechar='|') 
          for row in reader:
            vals = row[0].split(",")
            point = [float(vals[0]), float(vals[1])]
            if int(vals[-1]) == 0:
              free_points.append(point)
            else: 
              hit_points.append(point)
  except IOError:
      print ("IOError: " + csvfile)
      sys.exit()
  
  csvfile.close()

  # Visualize the hit points
  hit_points = np.asarray(hit_points)
  free_points = np.asarray(free_points)
  plt.scatter(hit_points[:,0], hit_points[:,1])
  # plt.scatter(free_points[:,0], free_points[:,1], c = 'r')
  plt.show()
