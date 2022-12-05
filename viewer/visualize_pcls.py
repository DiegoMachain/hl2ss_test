#!/usr/bin/env python3

import glob
# import numpy as np
import open3d as o3d
import re

folder = "./pcls/"

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

class Visualizer():
    def __init__(self):

        files = glob.glob("%s*.ply" % folder)
        files.sort(key=natural_keys)

        c = 0

        # read in all the files
        for i, file in enumerate(files):

            if c < (len(files)-3):

                # read ply file
                pcd1 = o3d.io.read_point_cloud(files[i])
                pcd2 = o3d.io.read_point_cloud(files[i+1])
                pcd3 = o3d.io.read_point_cloud(files[i+2])

                print("i: %s" % i)

                # # visualize and save a couple of screenshots
                o3d.visualization.draw_geometries([pcd1, pcd2, pcd3])

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
