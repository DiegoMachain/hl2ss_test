#!/usr/bin/env python
#Crop the point clouds
# examples/python/visualization/interactive_visualization.py

import numpy as np
import copy
import open3d as o3d


def demo_crop_geometry(pcd):
    print("Demo for manual geometry cropping")
    print(
        "1) Press 'Y' twice to align geometry with negative direction of y-axis"
    )
    print("2) Press 'K' to lock screen and to switch to selection mode")
    print("3) Drag for rectangle selection,")
    print("   or use ctrl + left click for polygon selection")
    print("4) Press 'C' to get a selected geometry")
    print("5) Press 'S' to save the selected geometry")
    print("6) Press 'F' to switch to freeview mode")

    o3d.visualization.draw_geometries_with_editing([pcd])

def pick_points(pcd):
    print("")
    print(
        "1) Please pick at least three correspondences using [shift + left click]"
    )
    print("   Press [shift + right click] to undo point picking")
    print("2) After picking points, press 'Q' to close the window")
    vis_2 = o3d.visualization.VisualizerWithEditing()
    vis_2.create_window()
    vis_2.add_geometry(pcd)
    vis_2.run()  # user picks points
    vis_2.destroy_window()
    print("")
    return vis_2.get_picked_points()
