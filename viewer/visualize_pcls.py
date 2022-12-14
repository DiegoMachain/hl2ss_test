#!/usr/bin/env python3

import glob
import open3d as o3d
import re
import argparse

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

# ------------------------------------------------------------------------------------------------------- #

if __name__ == '__main__':

    argParser = argparse.ArgumentParser()
    argParser.add_argument("-f", "--folder", help="path to folder with ply files")
    argParser.add_argument("-p", "--pattern", help="part of file names to match to extract correct ones, e.g. world")

    args = argParser.parse_args()

    files = glob.glob("%s%s*.ply" % (args.folder, args.pattern))
    files.sort(key=natural_keys)

    c = 0
    step = 1

    # read in all the files
    for i, file in enumerate(files):

        if c < (len(files)-(3*step)):

            # read ply file
            pcd1 = o3d.io.read_point_cloud(files[i])
            pcd2 = o3d.io.read_point_cloud(files[i+1*step])
            pcd3 = o3d.io.read_point_cloud(files[i+2*step])
            print("c: %s" % c)

            points = [[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]]
            lines = [[0, 1], [0, 2], [0, 3]]
            colors = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
            line_set = o3d.geometry.LineSet()
            line_set.points = o3d.utility.Vector3dVector(points)
            line_set.lines = o3d.utility.Vector2iVector(lines)
            line_set.colors = o3d.utility.Vector3dVector(colors)

            # visualize three point clouds at once
            o3d.visualization.draw_geometries([pcd1, pcd2, pcd3, line_set])

            c += 1
