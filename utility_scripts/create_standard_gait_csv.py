import numpy as np
import matplotlib.pyplot as plt
import os
import bezier 
from scipy.interpolate import interp1d

TIMESTEPS = 40 # Number of time steps for each step, note that first and last step are half the size

def make_evenly_spaced_points(x, z, array_size):
    # Create an array of cumulative distances between points
    distances = np.sqrt(np.diff(x)**2 + np.diff(z)**2)
    cumulative_distances = np.insert(np.cumsum(distances), 0, 0)

    # Generate new evenly spaced distances
    even_distances = np.linspace(cumulative_distances[0], cumulative_distances[-1], num=array_size)

    # Interpolate the x and z values over the new distances
    interp_x = interp1d(cumulative_distances, x, kind='cubic')
    interp_z = interp1d(cumulative_distances, z, kind='cubic')

    x_new = interp_x(even_distances)
    z_new = interp_z(even_distances)

    return x_new, z_new

def package_bezier(step_length, array_size):

    xzpositions_first_step = np.asfortranarray([[0.0, (0.02/0.1)*(step_length/2), (0.0533/0.1)*(step_length/2), (step_length/2)], [0.0, 0.4, 0.2, 0.0]])
    curvexz_first_step = bezier.Curve(xzpositions_first_step, degree=3)
    number_of_time_points_first_step = np.linspace(0, 1.0, int(array_size/2))
    points_first_step= curvexz_first_step.evaluate_multi(number_of_time_points_first_step)
    x_swing_first_step = points_first_step[0,:]
    z_swing_first_step = points_first_step[1,:]
    x_stance_first_step = np.linspace(0, 0 - (step_length/2), int(array_size/2))
    # z_stance_first_step = compensation_for_circle(int(array_size/2), step_length/2)
    z_stance_first_step = [0]*int(array_size/2)
    # z_swing_first_step = z_swing_first_step + z_stance_first_step

    x_swing_first_step, z_swing_first_step = make_evenly_spaced_points(x_swing_first_step, z_swing_first_step, int(array_size/2))

    final_points_first_step = np.column_stack((x_swing_first_step, z_swing_first_step, x_stance_first_step, z_stance_first_step))
    plt.scatter(x_swing_first_step, z_swing_first_step)
    plt.plot(x_stance_first_step, z_stance_first_step, color="orange")
    plt.show()
    # print(final_points.shape)

    xzpositions_complete_step = np.asfortranarray([[0.0, (0.02/0.1)*step_length, (0.0533/0.1)*step_length, step_length], [0.0, 0.4, 0.2, 0.0]])
    curvexz_complete_step = bezier.Curve(xzpositions_complete_step, degree=3)
    number_of_time_points_complete_step = np.linspace(0, 1.0, array_size)
    points_complete_step= curvexz_complete_step.evaluate_multi(number_of_time_points_complete_step)
    x_swing_complete_step = points_complete_step[0,:] - (step_length/2)
    z_swing_complete_step = points_complete_step[1,:]
    x_stance_complete_step = np.linspace(0+(step_length/2), 0 - (step_length/2), array_size)
    # z_stance_complete_step = compensation_for_circle(array_size, step_length)
    z_stance_complete_step = [0]*array_size
    # z_swing_complete_step = z_swing_complete_step + z_stance_complete_step

    x_swing_complete_step, z_swing_complete_step = make_evenly_spaced_points(x_swing_complete_step, z_swing_complete_step, array_size)

    final_points_complete_step = np.column_stack((x_swing_complete_step, z_swing_complete_step, x_stance_complete_step, z_stance_complete_step))
    plt.scatter(x_swing_complete_step, z_swing_complete_step)
    plt.plot(x_stance_complete_step, z_stance_complete_step, color="orange")
    plt.show()

    return final_points_first_step, final_points_complete_step

large_gait_first_step, large_gait_complete_step = package_bezier(0.6, TIMESTEPS)
np.savetxt('ros2/src/march_gait_planning/m9_gait_files/cartesian/first_step_large.csv', large_gait_first_step, delimiter=',')
np.savetxt('ros2/src/march_gait_planning/m9_gait_files/cartesian/normal_gait_large.csv', large_gait_complete_step, delimiter=',')
small_gait_first_step, small_gait_complete_step = package_bezier(0.2, TIMESTEPS)
np.savetxt('ros2/src/march_gait_planning/m9_gait_files/cartesian/first_step_small.csv', small_gait_first_step, delimiter=',')
np.savetxt('ros2/src/march_gait_planning/m9_gait_files/cartesian/normal_gait_small.csv', small_gait_complete_step, delimiter=',')

def interpolate(step_size, small_gait_complete_step, large_gait_complete_step, array_size):
    z_swing = []
    x_swing = np.linspace(0, step_size/2, array_size).tolist()
    for i in range(array_size):
        try:  
            z = small_gait_complete_step[i][1] + (x_swing[i]-small_gait_complete_step[i][0]) * ((large_gait_complete_step[i][1]-small_gait_complete_step[i][1])/(large_gait_complete_step[i][0]-small_gait_complete_step[i][0]))
            z_swing.append(z)
        except RuntimeWarning:
            z_swing.append(0.0)
    x_stance = np.linspace(0.0, -step_size/2, array_size).tolist()
    z_stance = np.linspace(0.0, 0.0, array_size).tolist()
    for k in range(array_size):
        try: 
            z = small_gait_complete_step[k][1] + (x_swing[k]-small_gait_complete_step[k][0]) * ((large_gait_complete_step[k][1]-small_gait_complete_step[k][1])/(large_gait_complete_step[k][0]-small_gait_complete_step[k][0]))
            z_stance.append(z)
        except RuntimeWarning: 
            z_stance.append(0.0)
        x_stance.append(x_swing[k]-step_size/2)
        x_swing.append(x_stance[k]+step_size/2)
        z_stance.append(z)
        z_swing.append(z_stance[k])

    result = np.column_stack((x_swing, z_swing, x_stance, z_stance))
    plt.plot(result[:,0], result[:,1])
    plt.plot(result[:,2], result[:,3])
    plt.plot(small_gait_complete_step[:,0], small_gait_complete_step[:,1])
    plt.plot(large_gait_complete_step[:,0], large_gait_complete_step[:,1])
    plt.show()
    return result 

# res = interpolate(0.3, small_gait_first_step, large_gait_first_step, 100)

# plt.plot(large_gait_complete_step[:,0], large_gait_complete_step[:,1])
# plt.plot(small_gait_complete_step[:,0], small_gait_complete_step[:,1])
# plt.show()

large_step_close, _ = package_bezier(-0.6, TIMESTEPS)
large_step_close = np.flip(large_step_close, axis=0)
np.savetxt('ros2/src/march_gait_planning/m9_gait_files/cartesian/large_step_close.csv', large_step_close, delimiter=',')

small_step_close, _ = package_bezier(-0.2, TIMESTEPS)
small_step_close = np.flip(small_step_close, axis=0)
np.savetxt('ros2/src/march_gait_planning/m9_gait_files/cartesian/small_step_close.csv', small_step_close, delimiter=',')