import open3d as o3d
import numpy as np
import copy


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
    print(points)
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
    

    """ #Create the vector and sphere
    #Transform back 
    sphere_pivot = vec
    center_transform = np.eye(4)
    center_transform[:3, 3] = pivot
    center_transform_2 = np.eye(4)
    center_transform_2[:3, 3] = center

    sphere = o3d.geometry.TriangleMesh.create_sphere(radius = .05)
    sphere.compute_vertex_normals()
    sphere.transform(center_transform)
    sphere.rotate(R, center=(0, 0, 0))
    sphere.scale(scale, np.zeros((3, 1)))

    sphere.transform(center_transform_2)

    vec_len = scale
    arrow = o3d.geometry.TriangleMesh.create_arrow(
        cone_height = 0.2 * vec_len,
        cone_radius = 0.06 * vec_len,
        cylinder_height = 0.8 * vec_len,
        cylinder_radius = 0.04 * vec_len
    )


    o3d.visualization.draw_geometries([arrow,ref_frame])
    arrow.compute_vertex_normals()
    arrow.rotate(rot_mat_2, center = (0,0,0))

    arrow.transform(center_transform)

    arrow.rotate(R, center = (0,0,0))
    arrow.scale(scale, np.zeros((3,1)))
    arrow.transform(center_transform_2) """

    #Compute rotation matrix based on the results of Ditto
    rot_mat_2 = vector_to_rotation(vec)

    arrow_vec = np.array([[0], [0], [1]])
    arrow_vec = rot_mat_2 @ arrow_vec

    #Move to pivot point 
    arrow_vec = pivot.reshape((3,1)) + arrow_vec

    #Rotate
    arrow_vec = R @ arrow_vec

    #Scale it 
    arrow_vec *= scale

    # move the center
    arrow_vec = center.reshape((3,1)) + arrow_vec

    #Line to draw on open3d
    arrow_2 = drawLines(sphere_point, arrow_vec)

    return arrow_vec, sphere_point, sphere_2, ref_frame, arrow_2