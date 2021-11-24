import open3d as o3d
import timeit
import numpy as np
from numpy.random import uniform    
import copy
from scipy.optimize import linprog
from scipy.spatial import Delaunay
from scipy.signal import convolve2d
from numpy.lib.stride_tricks import sliding_window_view

s0 = timeit.default_timer()

NUMBER_OF_NEIGHBOURS = 5
MINIMUM_POINTS_IN_REGION = 20
ANGLE_DIFF_THRESHOLD = np.cos(20 * np.pi / 180)
ALLOWED_CURVATURE = np.cos(35 * np.pi / 180)

X1 = 0.2
X2 = 0.7
Y = 0.4
Z1 = 0.5
Z2 = 0

start = timeit.default_timer()
pcd = o3d.io.read_point_cloud("datasets/stairs2.ply")
orig_pcd = pcd
print('IO time:\t\t\t ', timeit.default_timer() - start)  

# Downsampling
start = timeit.default_timer()
pcd = pcd.voxel_down_sample(voxel_size=0.01)
print('Down sampling time:\t\t ', timeit.default_timer() - start)   

# hiqp base to hip aa
pcd = pcd.translate(-1 * np.array([0, 0.0725, 0])) 
pcd = pcd.rotate(pcd.get_rotation_matrix_from_xyz((0.0756695435357118, 0, 0)), np.zeros(3))

# hip aa to upper leg
pcd = pcd.translate(-1 * np.array([-0.151, 0, 0])) 
pcd = pcd.rotate(pcd.get_rotation_matrix_from_xyz((0, 0.000989348615771038, 0)), np.zeros(3))

# upper leg to lower leg
pcd = pcd.translate(-1 * np.array([0, -0.03, -0.41])) 
pcd = pcd.rotate(pcd.get_rotation_matrix_from_xyz((0, -0.009588377793440255, 0)), np.zeros(3))

# lower leg to ankle plate
pcd = pcd.translate(-1 * np.array([0, -0.08, -0.39])) 
pcd = pcd.rotate(pcd.get_rotation_matrix_from_xyz((0, -0.1330429972669268, 0)), np.zeros(3))

# Normal estimation
start = timeit.default_timer()
pcd.estimate_normals() 
print('Normal estimation time:\t\t ', timeit.default_timer() - start)  

# Downsampling
start = timeit.default_timer()
pcd = pcd.voxel_down_sample(voxel_size=0.04)
print('Down sampling time:\t\t ', timeit.default_timer() - start)  


# Filter points
start = timeit.default_timer()
# pcd = pcd.rotate(pcd.get_rotation_matrix_from_xyz((np.pi, np.pi, np.pi)), np.zeros(3))


points = np.asarray(pcd.points)
mask = np.where((points[:,0] > -X1) & (points[:, 0] < X2) & (points[:, 1] > -Y) & (points[:, 1] < Y) & (points[:, 2] > -Z1) & (points[:, 2] < Z2))[0]
pcd.points = o3d.utility.Vector3dVector(points[mask]) # normals and colors are unchanged
pcd.colors = o3d.utility.Vector3dVector(np.asarray(pcd.colors)[mask])
pcd.normals = o3d.utility.Vector3dVector(np.asarray(pcd.normals)[mask])
print('Filter time:\t\t\t ', timeit.default_timer() - start) 

# Normalize vectors
start = timeit.default_timer()
pcd = pcd.normalize_normals()
print('Normalization time:\t\t ', timeit.default_timer() - start)  

# pcd = pcd.translate(np.array([-.6, 0, 1]))
# pcd = pcd.rotate(pcd.get_rotation_matrix_from_xyz((0, np.pi/2, 0)), np.zeros(3))
# pcd = pcd.rotate(pcd.get_rotation_matrix_from_xyz((0, 0, np.pi/8)), np.zeros(3))

# Default colors-
np.asarray(pcd.colors)[:, :] = [0, 0, 0]  # All black
np.asarray(orig_pcd.colors)[:, :] = [0.95, 0.95, 0.95]  # Light gray

# KDTree fitting
start = timeit.default_timer()
pcd_tree = o3d.geometry.KDTreeFlann(pcd)
print('Tree creation time:\t\t ', timeit.default_timer() - start)  

# Calculate curvature values
z_unit = np.array([0.6184282, -0.13936759, -0.76996126])
normals = np.copy(pcd.normals)
normals *= z_unit
dot_products = np.abs(np.sum(normals, axis=1))

print('Calculate dot products:\t\t ', timeit.default_timer() - start)  
start = timeit.default_timer()

flat_regions = []
region = []
checked = set()
queue = set()
indices = np.arange(len(pcd.points))

start = timeit.default_timer()
counter = 0

while len(checked) < len(indices):
    if len(queue) > 0:
        current_index = queue.pop()
    else:
        if len(region) > MINIMUM_POINTS_IN_REGION:
            flat_regions.append(region)
        region = []
        current_index = np.nanargmin(dot_products)
        dot_products[current_index] = np.nan
        region.append(current_index)
    
    if current_index not in checked:
        checked.add(current_index)
    
    # Compute angle of current point normal with Z-normal
    normal = pcd.normals[current_index]
        
    [k, idx, _] = pcd_tree.search_knn_vector_3d(pcd.points[current_index], NUMBER_OF_NEIGHBOURS)

    for n_idx in idx[1:]:
        neighbour_normal = pcd.normals[n_idx]
        dot = np.abs(np.dot(normal, neighbour_normal))    

        if dot >= ANGLE_DIFF_THRESHOLD:
            if (n_idx not in checked) and (n_idx not in queue):
                region.append(current_index)
                if ALLOWED_CURVATURE <= dot_products[n_idx] <= 1:
                    queue.add(n_idx)


print('Region finding time:\t\t ', timeit.default_timer() - start)  


start = timeit.default_timer()

hulls = []
linesets = []
meshes = []
planes = []
clouds = []
rotation_matrices = []
plane_heights = []
colors = []
total_cells = 0


for region in flat_regions:
    total_cells += len(region)
    R  = uniform(0.6, 1)
    G = uniform(0.6, 1)
    B = uniform(0.6, 1)
    np.asarray(pcd.colors)[region, :] = [R, G, B]
    colors.append([R, G, B])

    pcd_region = copy.deepcopy(pcd)
    points = np.asarray(pcd_region.points)
    pcd_region.points = o3d.utility.Vector3dVector(points[np.array(region)])
    # pcd_region = pcd_region.voxel_down_sample(voxel_size=0.07)
    plane_model, inliers = pcd_region.segment_plane(0.005, 4, 200)

    planes.append(plane_model)
    [a, b, c, d] = plane_model 

    points = np.asarray(pcd_region.points)
    z = -(np.sum(np.array([a, b, 0]) * points, axis=1) + d)/c
    np.asarray(pcd_region.points)[:, 2] = z
        
    normal = np.array([a, b, c])

    z_unit = [0, 0, 1]
    costheta = np.dot(normal, z_unit)
    axis = np.cross(normal, z_unit) / np.linalg.norm(np.cross(normal, z_unit))
    s = np.sqrt(1 - costheta * costheta)
    C = 1 - costheta
    x = axis[0]
    y = axis[1]
    z = axis[2]
    R = np.array([[ x*x*C+c,    x*y*C-z*s,  x*z*C+y*s ],
                  [ y*x*C+z*s,  y*y*C+c,    y*z*C-x*s ],
                  [ z*x*C-y*s,  z*y*C+x*s,  z*z*C+c   ]])  

    points = np.asarray(pcd_region.points)
    points = np.dot(points, R.T)
    pcd_region.points = o3d.utility.Vector3dVector(points)

    plane_heights.append(pcd_region.points[0][2])
    
    hull = Delaunay(np.asarray(pcd_region.points)[:, 0:2], 'Qx')
    hulls.append(hull)
    clouds.append(pcd_region)
    rotation_matrices.append(R)


print('Hull finding time:\t\t ', timeit.default_timer() - start)

start = timeit.default_timer()


intersection_clouds = []

# Draw lines
rays = []
masks = []
hull_meshes = []

ray_direction = np.array([0.6184282, -0.13936759, -0.76996126])
ray_direction = np.array([1.4, 0.3, -1.2])
# ray_direction = np.array([0, 0, -1])
for i in range(len(hulls)):
    
    hull = hulls[i]
    [a, b, c, d] = planes[i]
    R = rotation_matrices[i]
    R_inv = np.linalg.inv(R)
    z = plane_heights[i]
    direction = np.dot(R, ray_direction)

    N = 7
    plane_points = []

    XX, YY = np.meshgrid(np.linspace(-.25, .25, N), np.linspace(-.25, .25, N))
    startpoints = np.stack([XX, YY, np.zeros_like((XX))], axis=-1).reshape((N * N, 3), order='F')
    tf_startpoints = np.dot(startpoints, R.T)
    t = (z - tf_startpoints[:, 2]) / direction[2]
    plane_points = tf_startpoints + np.c_[t, t, t] * direction
    
    def in_hull(vector):
        return 1 if hull.find_simplex(vector[:2]) >= 0 else 0
            
    mask = np.apply_along_axis(in_hull, 1, plane_points).reshape(N, N)

    plane_points = np.dot(plane_points, R_inv.T)

    # for k in range(len(values)):
    #     for l in range(len(values)):
    #         # Plot rays
    #         startpoint = np.dot(R, [values[k], values[l], 0])
    #         # line_set = o3d.geometry.LineSet(
    #         #     points=o3d.utility.Vector3dVector([[values[k], values[l], 0], [values[k], values[l], 0] + 0.4 * ray_direction]),
    #         #     lines=o3d.utility.Vector2iVector([[0, 1]]),
    #         # )
    #         # line_set.colors = o3d.utility.Vector3dVector([[1.0, 0.1, 0.1]])
    #         # rays.append(line_set)
    #         t = (z - startpoint[2]) / direction[2]
    #         intersection = startpoint + t * direction
    #         if hull.find_simplex(intersection[:2]) >= 0:
    #             step_mask[k][l] = 1
    #         plane_points.append(np.dot(R_inv, intersection))

    masks.append(mask)
    width = 2
    height = 4
    convolutions = convolve2d(mask, np.ones((height, width)), 'valid')
    x, y = np.where(convolutions == height * width)
    valid_indices = set()
    valid_points = []
    for j in range(len(x)):
        for w in range(0, width):
            for h in range(0, height):
                index = (y[j] + w) + N * (x[j] + h)
                if index not in valid_indices:
                    valid_points.append(plane_points[index])
                    valid_indices.add(index)
   
    intersection_cloud = o3d.geometry.PointCloud()
    intersection_cloud.points = o3d.utility.Vector3dVector(valid_points)
    intersection_cloud.paint_uniform_color([0.8, 0.2, 0.2])

    intersection_clouds.append(intersection_cloud)

    hull_mesh = o3d.geometry.TriangleMesh()   
    points = np.c_[hull.points, np.full(len(hull.points), z)]
    points = np.dot(points, R_inv.T)
    hull_mesh.vertices = o3d.utility.Vector3dVector(points)
    hull_mesh.triangles = o3d.utility.Vector3iVector(hull.simplices)
    hull_mesh.paint_uniform_color(colors[i])
    hull_meshes.append(hull_mesh)

start = timeit.default_timer()
mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=[0, 0, 0])

print('Ray tracing time:\t\t ', timeit.default_timer() - start)

print('Total running time:\t\t ', timeit.default_timer() - s0)

points = [
    [-X1, -Y, Z2],
    [X2, -Y, Z2],
    [-X1, Y, Z2],
    [X2, Y, Z2],
    [-X1, -Y, -Z1],
    [X2, -Y, -Z1],
    [-X1, Y, -Z1],
    [X2, Y, -Z1],
]
lines = [
    [0, 1],
    [0, 2],
    [1, 3],
    [2, 3],
    [4, 5],
    [4, 6],
    [5, 7],
    [6, 7],
    [0, 4],
    [1, 5],
    [2, 6],
    [3, 7],
]
colors = [[0.7, 0.7, 0.7] for i in range(len(lines))]
line_set = o3d.geometry.LineSet(
    points=o3d.utility.Vector3dVector(points),
    lines=o3d.utility.Vector2iVector(lines),
)
line_set.colors = o3d.utility.Vector3dVector(colors)

# o3d.visualization.draw_geometries([pcd])
o3d.visualization.draw_geometries([line_set] + hull_meshes + intersection_clouds, mesh_show_back_face=True)
# o3d.visualization.draw_geometries([mesh_frame, line_set] + clouds + rays + intersection_clouds)
# o3d.visualization.draw_geometries([mesh_frame, line_set] + clouds + rays)
# o3d.visualization.draw_geometries([pcd, mesh_frame, line_set] + meshes)
# o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)
