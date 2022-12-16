import glob
import numpy as np
import open3d as o3d
import argparse
import csv
import sys
import pandas as pd
import matplotlib.pyplot as plt

def import_csv(path):
    max_x = 0
    max_y = 0
    means = []
    variances = []
    #open file
    with open(path, "r") as csvfile:
        reader = csv.reader(csvfile)
        for i, line in enumerate(reader):
            if float(line[0])>max_x:
                max_x = float(line[0])
            if float(line[1])>max_y:
                max_y = float(line[1])
            means.append(line[2])
            #print(means[i])
            variances.append(line[3])
    return max_x, max_y, means, variances

def write_ros(path, x, y, means, variances):
    with open(path, "w") as f:
        f.write("rostopic pub /mean nav_msgs/OccupancyGrid \"{info: {width: ")
        f.write(str(int(x)))
        f.write(", height: ")
        f.write(str(int(y)))
        f.write("}, data: [")
        f.write(str(means[0]))
        for index in range(len(means)-1):
            f.write(", ")
            f.write(means[index + 1])
        f.write("]}\" \n")

        f.write("rostopic pub /variance nav_msgs/OccupancyGrid \"{info: {width: ")
        f.write(str(int(x)))
        f.write(", height: ")
        f.write(str(int(y)))
        f.write("}, data: [")
        f.write(str(variances[0]))
        for index in range(len(variances)-1):
            f.write(", ")
            f.write(variances[index + 1])
        f.write("]}\" \n")
    return True
            
  


if __name__ == '__main__':
    argParser = argparse.ArgumentParser()
    argParser.add_argument("-f", "--file", help="path to csv file")
    argParser.add_argument("-o", "--output", help="path to output folder")
    
    args = argParser.parse_args()

    path_in = args.file
    path_out = args.output

    width, height, means_array, variances_array = import_csv(path_in)

    print(means_array)

    if  write_ros(path_out, width, height, means_array, variances_array):
        print("ROS messages located at: ")
        print(str(path_out))
