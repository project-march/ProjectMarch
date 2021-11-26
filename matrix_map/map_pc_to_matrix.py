import open3d as o3d
import timeit
import numpy as np
from numpy.random import uniform    
import copy
from scipy.optimize import linprog
from scipy.spatial import Delaunay
from scipy.signal import convolve2d
from numpy.lib.stride_tricks import sliding_window_view
from matplotlib import pyplot as plt

s0 = timeit.default_timer()

NUMBER_OF_NEIGHBOURS = 5
MINIMUM_POINTS_IN_REGION = 20
ANGLE_DIFF_THRESHOLD = np.cos(20 * np.pi / 180)
ALLOWED_CURVATURE = np.cos(35 * np.pi / 180)

X1 = 0.5
X2 = 0.5
Y = 0.5
Z1 = 0.5
Z2 = 0.5

start = timeit.default_timer()
pcd = o3d.io.read_point_cloud("datasets/stairs4.ply")
orig_pcd = pcd
print('IO time:\t\t\t ', timeit.default_timer() - start)  

# Downsampling
start = timeit.default_timer()
# pcd = pcd.voxel_down_sample(voxel_size=0.005)
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
# pcd = pcd.voxel_down_sample(voxel_size=0.005)
print('Down sampling time:\t\t ', timeit.default_timer() - start)  


# Filter points
start = timeit.default_timer()
# pcd = pcd.rotate(pcd.get_rotation_matrix_from_xyz((np.pi, np.pi, np.pi)), np.zeros(3))

# pcd = pcd.translate(np.array([-.6, 0, 1]))
pcd = pcd.rotate(pcd.get_rotation_matrix_from_xyz((np.pi, 0, 0)), np.zeros(3))
pcd = pcd.rotate(pcd.get_rotation_matrix_from_xyz((0, 0, 0.5*np.pi)), np.zeros(3))
pcd = pcd.rotate(pcd.get_rotation_matrix_from_xyz((0, -np.pi/8, 0)), np.zeros(3))
pcd = pcd.rotate(pcd.get_rotation_matrix_from_xyz((-np.pi/4 - np.pi/2, 0, 0)), np.zeros(3))

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

# Default colors-
# np.asarray(pcd.colors)[:, :] = [0, 0, 0]  # All black
# np.asarray(orig_pcd.colors)[:, :] = [0.95, 0.95, 0.95]  # Light gray

# KDTree fitting
start = timeit.default_timer()
pcd_tree = o3d.geometry.KDTreeFlann(pcd)
print('Tree creation time:\t\t ', timeit.default_timer() - start)  

mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=[0, 0, 0])
o3d.visualization.draw_geometries([pcd, mesh_frame], mesh_show_back_face=True)

N = 50

arr = np.full((N, N), 0.0)

start = timeit.default_timer()

for point in np.asarray(pcd.points):
    x = point[0]
    y = point[1]
    z = point[2]

    x_index = int((x + 0.5) * N)
    y_index = int((y + 0.5) * N)

    if arr[x_index, y_index] == 0.0:
        arr[x_index, y_index] = z
    else:
        arr[x_index, y_index] = (arr[x_index, y_index] + z) / 2

print('Map generation time:\t\t ', timeit.default_timer() - start)
start = timeit.default_timer()

arr = np.rot90(arr)

laplacian = np.array([[0, -1, 0],
                      [-1, 4, -1],
                      [0, -1, 0]])

clip_max = 0.2
clip_min = 0.05

derivatives = np.abs(convolve2d(arr, laplacian, 'same'))

print('Laplacian time:\t\t ', timeit.default_timer() - start)

derivatives[derivatives > clip_max] = clip_max
derivatives[derivatives < clip_min] = 0
        

plt.imshow(arr, interpolation='none', extent=[-0.5, 0.5, -0.5, 0.5])
plt.colorbar()
threshold = 0.06
plt.show()

cellwidth = 1 / N

shift = 0

feasible_positions = []

for t in range(0, 10):
    if t % 2 == 1:
        shift += 1
    shift *= -1

    plt.imshow((derivatives), interpolation='none', extent=[-0.5, 0.5, -0.5, 0.5])
    plt.colorbar()

    S = 0
    x_opt = int((0.2 + 0.5) * N)
    y_opt = int((0.15 + 0.5) * N) - shift

    for x in range(x_opt - 2, x_opt + 3):
        for y in range(y_opt - 4, y_opt + 5):
            val = derivatives[y, x]
            if val < threshold:
                S += 1
                plt.scatter((x / N) - 0.5 + cellwidth/2, ((N - y) / N) - 0.5 - cellwidth/2, c='r', s=5, marker='s')
    
    if S == 9 * 5:
        z = arr[y_opt, x_opt]
        feasible_positions.append([(x_opt / N) - 0.5 + cellwidth/2, ((N - y_opt) / N) - 0.5 - cellwidth/2, z])
        plt.show()

    plt.clf()

print(feasible_positions)

# plt.imshow(np.rot90(derivatives), interpolation='none', extent=[-0.5, 0.5, -0.5, 0.5])

# for i in range(0, N - 2):
#     for j in range(0, N - 2):
#         if derivatives[i, j] < threshold:
#             plt.scatter(i / (N-2) - 0.5, j / (N-2) - 0.5, c='r')


