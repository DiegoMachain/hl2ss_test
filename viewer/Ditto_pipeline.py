import os, sys
sys.path.append('../')
os.environ["PYOPENGL_PLATFORM"] = "egl"

import numpy as np
import json
import math

import torch

import open3d as o3d
import json
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from src.utils.misc import sample_point_cloud


#Packages for the visualization
import trimesh
from src.utils.joint_estimation import aggregate_dense_prediction_r
import pymeshfix
from utils3d.mesh.utils import as_mesh

def plot_3d_point_cloud(x,
                        y,
                        z,
                        show=True,
                        show_axis=True,
                        in_u_sphere=False,
                        marker='.',
                        s=8,
                        alpha=.8,
                        figsize=(5, 5),
                        elev=10,
                        azim=240,
                        axis=None,
                        title=None,
                        lim=None,
                        *args,
                        **kwargs):

    if axis is None:
        fig = plt.figure(figsize=figsize)
        ax = fig.add_subplot(111, projection='3d')
    else:
        ax = axis
        fig = axis

    if title is not None:
        plt.title(title)

    sc = ax.scatter(x, y, z, marker=marker, s=s, alpha=alpha, *args, **kwargs)
    ax.view_init(elev=elev, azim=azim)

    if lim:
        ax.set_xlim3d(*lim[0])
        ax.set_ylim3d(*lim[1])
        ax.set_zlim3d(*lim[2])
    elif in_u_sphere:
        ax.set_xlim3d(-0.5, 0.5)
        ax.set_ylim3d(-0.5, 0.5)
        ax.set_zlim3d(-0.5, 0.5)
    else:
        lim = (min(np.min(x), np.min(y),
                   np.min(z)), max(np.max(x), np.max(y), np.max(z)))
        ax.set_xlim(1.3 * lim[0], 1.3 * lim[1])
        ax.set_ylim(1.3 * lim[0], 1.3 * lim[1])
        ax.set_zlim(1.3 * lim[0], 1.3 * lim[1])
        plt.tight_layout()

    if not show_axis:
        plt.axis('off')

    if show:
        plt.show()

    return fig

def vector_to_rotation(vector):
    z = np.array(vector)
    z = z / np.linalg.norm(z)
    x = np.array([1, 0, 0])
    x = x - z*(x.dot(z)/z.dot(z))
    x = x / np.linalg.norm(x)
    y = np.cross(z, x)
    return np.c_[x, y, z]

def sum_downsample_points(point_list, voxel_size=0.01, nb_neighbors=20, std_ratio=2.0):
    points = np.concatenate([np.asarray(x.points) for x in point_list], axis=0)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
    pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
    return pcd

def normalize(tensor: torch.Tensor, dim: int) -> torch.Tensor:
    return tensor / ((tensor ** 2).sum(dim, keepdim=True).sqrt() + 1.0e-5)

#Function to fix the mesh if it has holes in the middle and the mobile and fixed parts are mostlycomposed by flat
#parts
def fixMesh(static_part, mobile_part):
    #Create static mesh

    static_pcd = o3d.geometry.PointCloud()
    static_pcd.points = o3d.utility.Vector3dVector(np.asarray(static_part.vertices))
    static_pcd = sum_downsample_points([static_pcd], 0.01, 50, 0.1)

    #Create mobile mesh to fix 
    mobile_pcd = o3d.geometry.PointCloud()
    mobile_pcd.points = o3d.utility.Vector3dVector(np.asarray(mobile_part.vertices))
    mobile_pcd = sum_downsample_points([mobile_pcd], 0.01, 50, 0.1)

    #Create point cloud
    alpha = .5
    print(f"alpha={alpha:.3f}")
    static_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(static_pcd, alpha)
    static_mesh.compute_vertex_normals()


    alpha = .5
    print(f"alpha={alpha:.3f}")
    mobile_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(mobile_pcd, alpha)
    mobile_mesh.compute_vertex_normals()

    #Improvement of meshes
    fixed_static_mesh = pymeshfix.MeshFix(np.asarray(static_mesh.vertices), np.asarray(static_mesh.triangles))
    fixed_static_mesh.repair(verbose=True, joincomp = True, remove_smallest_components=False)
    fixed_static_mesh = trimesh.Trimesh(vertices = fixed_static_mesh.v, faces = fixed_static_mesh.f)


    fixed_mobile_mesh = pymeshfix.MeshFix(np.asarray(mobile_mesh.vertices), np.asarray(mobile_mesh.triangles))
    fixed_mobile_mesh.repair(verbose=True, joincomp = True, remove_smallest_components=False)
    fixed_mobile_mesh = trimesh.Trimesh(vertices = fixed_mobile_mesh.v, faces = fixed_mobile_mesh.f)

    #change the color of the mobile part
    fixed_mobile_mesh.visual.face_colors = np.array([84, 220, 83, 255], dtype=np.uint8)
    

    return fixed_static_mesh, fixed_mobile_mesh

def add_r_joint_to_scene(scene,
                             axis,
                             pivot_point,
                             length,
                             radius=0.01,
                             joint_color=[200, 0, 0, 180],
                             recenter=False):
    if recenter:
        pivot_point = np.cross(axis, np.cross(pivot_point, axis))
    rotation_mat = vector_to_rotation(axis)
    screw_tran = np.eye(4)
    screw_tran[:3, :3] = rotation_mat
    screw_tran[:3, 3] = pivot_point
    
    axis_cylinder = trimesh.creation.cylinder(radius, height=length)
    axis_arrow = trimesh.creation.cone(radius * 2, radius * 4)
    arrow_trans = np.eye(4)
    arrow_trans[2, 3] = length / 2
    axis_arrow.apply_transform(arrow_trans)
    axis_obj = trimesh.Scene((axis_cylinder, axis_arrow))
    screw = as_mesh(axis_obj)
    
    # screw.apply_translation([0, 0, 0.1])
    screw.apply_transform(screw_tran)
    screw.visual.face_colors = np.array(joint_color, dtype=np.uint8)
    scene.add_geometry(screw)
    return screw

def getCoordinateSystem(size):
    points = [[0, 0, 0], [size, 0, 0], [0, size, 0], [0, 0, size]]
    lines = [[0, 1], [0, 2], [0, 3]]
    colors = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(colors)
    return line_set


def Ditto(src_pcd_list, dst_pcd_list, model, generator, device):

    #Downsample the pointclouds
    src_pcd = sum_downsample_points(src_pcd_list, 0.005, 10, 2)
    dst_pcd = sum_downsample_points(dst_pcd_list, 0.005, 10, 2)

    # visualize crop results
    # tune crop box to get better isolated objects


    pc_start = np.asarray(src_pcd.points)
    pc_end = np.asarray(dst_pcd.points)

    bound_max = np.maximum(pc_start.max(0), pc_end.max(0))
    bound_min = np.minimum(pc_start.min(0), pc_end.min(0))
    norm_center = (bound_max + bound_min) / 2
    norm_scale = (bound_max - bound_min).max() * 1.1
    pc_start = (pc_start - norm_center) / norm_scale
    pc_end = (pc_end - norm_center) / norm_scale

    pc_start, _ = sample_point_cloud(pc_start, 8192)
    pc_end, _ = sample_point_cloud(pc_end, 8192)
    sample = {
        'pc_start': torch.from_numpy(pc_start).unsqueeze(0).to(device).float(),
        'pc_end': torch.from_numpy(pc_end).unsqueeze(0).to(device).float()
    }

    mesh_dict, mobile_points_all, c, stats_dict = generator.generate_mesh(sample)
    with torch.no_grad():
        joint_type_logits, joint_param_revolute, joint_param_prismatic = model.model.decode_joints(mobile_points_all, c)

    # compute articulation model
    mesh_dict[1].visual.face_colors = np.array([84, 220, 83, 255], dtype=np.uint8)
    joint_type_prob = joint_type_logits.sigmoid().mean()

    if joint_type_prob.item()< 0.5:
        # axis voting
        joint_r_axis = (
            normalize(joint_param_revolute[:, :, :3], -1)[0].cpu().numpy()
        )
        joint_r_t = joint_param_revolute[:, :, 3][0].cpu().numpy()
        joint_r_p2l_vec = (
            normalize(joint_param_revolute[:, :, 4:7], -1)[0].cpu().numpy()
        )
        joint_r_p2l_dist = joint_param_revolute[:, :, 7][0].cpu().numpy()
        p_seg = mobile_points_all[0].cpu().numpy()

        pivot_point = p_seg + joint_r_p2l_vec * joint_r_p2l_dist[:, np.newaxis]
        (
            joint_axis_pred,
            pivot_point_pred,
            config_pred,
        ) = aggregate_dense_prediction_r(
            joint_r_axis, pivot_point, joint_r_t, method="mean"
        )
    # prismatic
    else:
        # axis voting
        joint_p_axis = (
            normalize(joint_param_prismatic[:, :, :3], -1)[0].cpu().numpy()
        )
        joint_axis_pred = joint_p_axis.mean(0)
        joint_p_t = joint_param_prismatic[:, :, 3][0].cpu().numpy()
        config_pred = joint_p_t.mean()
        
        pivot_point_pred = mesh_dict[1].bounds.mean(0)


    static_part = mesh_dict[0].copy()
    mobile_part = mesh_dict[1].copy()
    fixed_static_mesh, fixed_mobile_mesh = fixMesh(static_part, mobile_part)
    
    #Original mesh
    scene = trimesh.Scene()
    scene_2 = trimesh.Scene()
    scene.add_geometry(static_part)
    scene.add_geometry(mobile_part)

    #Fixed mesh
    scene_2.add_geometry(fixed_static_mesh)
    scene_2.add_geometry(fixed_mobile_mesh)


    add_r_joint_to_scene(scene, joint_axis_pred, pivot_point_pred, 1.0, recenter=True)
    add_r_joint_to_scene(scene_2, joint_axis_pred, pivot_point_pred, 1.0, recenter=True)

    scene.show()
    scene_2.show()

    #We still need to return to the previous space
    
    return joint_axis_pred, pivot_point_pred

