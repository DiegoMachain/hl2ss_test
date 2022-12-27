import glob
import numpy as np
import open3d as o3d
import argparse
import csv
import sys
import pandas as pd
import matplotlib.pyplot as plt
import math

if __name__ == '__main__':
    argParser = argparse.ArgumentParser()
    argParser.add_argument("-f", "--file", help="path to csv file")
    argParser.add_argument("-o", "--output", help="output file of dataset")
    argParser.add_argument("-a", "--angle", help="counterclockwise rotation in degrees")   
    args = argParser.parse_args()   
    new_points = []     
    theta = (float(args.angle)/360.0)*2*math.pi  
    try:
        with open(args.file) as csvfile:
            reader = csv.reader(csvfile, delimiter=' ', quotechar='|') 
            for row in reader:
              vals = row[0].split(",")
              point = [float(vals[0]), float(vals[1])]

              newpoint = [(point[0]*math.cos(theta) - point[1]*math.sin(theta)),
                          (point[0]*math.sin(theta) + point[1]*math.cos(theta)),
                          int(vals[-1])]
              new_points.append(newpoint)   
    except IOError:
        print ("IOError: " + csvfile)
        sys.exit()

    csvfile.close()

    writefile = open('%s' % args.output, 'w')
    writer = csv.writer(writefile)

    for j in range(len(new_points)):
        data = [str(new_points[j][0]), str(new_points[j][1]), str(new_points[j][2])]
        writer.writerow(data)

    writefile.close()

    print("dataset rotated by %s degrees saved as %s" % (args.angle, args.output))


