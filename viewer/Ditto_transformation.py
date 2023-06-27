import open3d as o3d
import numpy as np
import copy
import math
from scipy.spatial.transform import Rotation as Rot

def quat_from_matrix(mat):
    q = np.empty((4,), dtype = np.float64)
    M = np.array(mat, dtype = np.float64, copy=False)[:4,:4]
    t = np.trace(M)

    if t > M[3, 3]:
        q[3] = t
        q[2] = M[1,0] - M[0,1]
        q[1] = M[0,2] - M[2,0]
        q[0] = M[2,1] - M[1,2]
    else:
        i,j,k = 0, 1, 2
        if M[1 ,1] > M[0,0]:
            i,j,k = 1,2,0
        if M[2,2] > M[i,i]:
            i,j,k = 2,0,1
        t = M[i,i] - (M[j,j] + M[k,k]) + M[3,3]
        q[i] = t
        q[j] = M[i,j] + M[j,i]
        q[k] = M[k,i] + M[i,k]
        q[3] = M[k,j] - M[j,k]
    q *= 0.5 / math.sqrt(t * M[3,3])
    return q

def getCoordinateSystem(size):
    points = [[0, 0, 0], [size, 0, 0], [0, size, 0], [0, 0, size]]
    lines = [[0, 1], [0, 2], [0, 3]]
    colors = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(colors)
    return line_set

def drawLines(pivot, dir):

    points = [[pivot[0], pivot[1], pivot[2]], [dir[0],dir[1], dir[2]]]
    lines = [[0, 1]]
    colors = [[1, 0, 0]]
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(colors)
    return line_set

def get_cross_prod_mat(pVec_Arr):

    mat = np.array([
        [0, -pVec_Arr[2], pVec_Arr[1]],
        [pVec_Arr[2], 0, -pVec_Arr[0]],
        [-pVec_Arr[1], pVec_Arr[0], 0]
    ])
    return mat
def getRotationMatrix(pVec_Arr):
    scale = np.linalg.norm(pVec_Arr)
    pVec_Arr /= scale

    z_unit_Arr = np.array([0,0,1])

    z_mat = get_cross_prod_mat(z_unit_Arr)

    z_vec = np.matmul(z_mat, pVec_Arr)
    z_vec_mat = get_cross_prod_mat(z_vec)

    if np.dot(z_unit_Arr, pVec_Arr) == -1:
        qTrans_mat = -np.eye(3,3)
    elif np.dot(z_unit_Arr, pVec_Arr) == 1:
        qTrans_mat = np.eye(3,3)
    else:
        qTrans_mat = np.eye(3,3) + z_vec_mat + np.matmul(z_vec_mat,z_vec_mat)/(1 +np.dot(z_unit_Arr, pVec_Arr))

    qTrans_mat *= scale

    qTrans_mat = o3d.utility.Vector3dVector(np.asarray(qTrans_mat))

    return qTrans_mat

def vector_to_rotation(vector):
    z = np.array(vector)
    z = z / np.linalg.norm(z)
    x = np.array([1, 0, 0])
    x = x - z*(x.dot(z)/z.dot(z))
    x = x / np.linalg.norm(x)
    y = np.cross(z, x)
    return np.c_[x, y, z]

def quat_from_matrix(mat):
    q = np.empty((4,), dtype = np.float64)
    M = np.array(mat, dtype = np.float64, copy=False)[:4,:4]
    t = np.trace(M)

    if t > M[3, 3]:
        q[3] = t
        q[2] = M[1,0] - M[0,1]
        q[1] = M[0,2] - M[2,0]
        q[0] = M[2,1] - M[1,2]
    else:
        i,j,k = 0, 1, 2
        if M[1 ,1] > M[0,0]:
            i,j,k = 1,2,0
        if M[2,2] > M[i,i]:
            i,j,k = 2,0,1
        t = M[i,i] - (M[j,j] + M[k,k]) + M[3,3]
        q[i] = t
        q[j] = M[i,j] + M[j,i]
        q[k] = M[k,i] + M[i,k]
        q[3] = M[k,j] - M[j,k]
    q *= 0.5 / math.sqrt(t * M[3,3])
    return q

#Function to transform Ditto results back to world coordinates
def transformDitto(pivot, vec, scale, center, sphere = True, refframe = True):

    #Translate sphere to pivot point to represent it
    sphere_point = pivot
    sphere_point = sphere_point.reshape((3,1))

    #Change the orientation of axis 
    R = np.array([[0,1,0],[0,0,1],[1,0,0]])
    sphere_point = R @ sphere_point
    sphere_point *= scale

    #Translate back
    sphere_point = center.reshape((3,1)) + sphere_point
    #Create another sphere to see if the points are well transformed
    if sphere:
        sphere_2 = o3d.geometry.TriangleMesh.create_sphere(radius = .05)
        sphere_2.compute_vertex_normals()

        sphere_2.paint_uniform_color([0.4,0.2,0.3])

        transform_sphere = np.eye(4)
        transform_sphere[:3, 3] = np.asarray(sphere_point.reshape((3,)))
        sphere_2.transform(transform_sphere)

    #Show the reference frame
    if refframe:
        ref_frame = getCoordinateSystem(1)
    

    rotation = np.zeros((4,4))
    rotation[3,3] = 1

    #Rotate back the vector
    #vec =  @ vec

    #Compute rotation matrix based on the results of Ditto
    rot_mat_2 = vector_to_rotation(vec)

    # theta = np.pi/ 2
    # R_y = np.array([[np.cos(theta), 0, np.sin(theta)], [0,1,0], [-np.sin(theta), 0, np.cos(theta)]])
    
    # arrow_vec_2 = np.array([[0], [0], [1]])
    # #arrow_vec_2 = rot_mat_2 @ R_y @ arrow_vec_2
    # arrow_vec_2 = rot_mat_2 @ arrow_vec_2

    # arrow_vec_2 = pivot.reshape((3,1)) + arrow_vec_2
    # arrow_vec_2 = R @ arrow_vec_2
    # arrow_vec_2 *= scale
    # # move the center
    # arrow_vec_2 = center.reshape((3,1)) + arrow_vec_2
    # arrow_vec_2 = arrow_vec_2 - sphere_point
    
    # rot_mat_3 = vector_to_rotation((arrow_vec_2).squeeze())

    # rotation[:3,:3] = rot_mat_3

    # r = Rot.from_matrix(rot_mat_3)
    quat = np.array([1.,1.,1.,1.])


    arrow_vec = np.array([[0], [0], [1]])
    arrow_vec = rot_mat_2 @ arrow_vec

    #Move to pivot point 
    arrow_vec = pivot.reshape((3,1)) + arrow_vec

    #Rotate
    arrow_vec = R @ arrow_vec


    #Convert the rotation amtrix to quaternions

    
    #Scale it 
    arrow_vec *= scale

    # move the center
    arrow_vec = center.reshape((3,1)) + arrow_vec

    diff = (arrow_vec - sphere_point).squeeze()
    quat[0] = diff[0]
    quat[1] = diff[1]
    quat[2] = diff[2]

    print("Vector Direction = ", quat)
    print("Vector Direction = ", arrow_vec - sphere_point)
    print("Vector Direction = ", diff)


    #Line to draw on open3d
    arrow_2 = drawLines(sphere_point, arrow_vec)

    return quat, sphere_point, sphere_2, ref_frame, arrow_2