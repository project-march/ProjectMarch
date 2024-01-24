import numpy as np
import matplotlib.pyplot as plt
import os
import bezier 


def cubic_bezier(array_size, step_length, x_curr):
    """
    Compute points on a cubic Bézier curve.
    
    Parameters:
        array_size (int): Number of points to generate on the curve.
        step_length (float): Step length for the curve in METERS.
        x_curr (tuple): Current foot position in xz-plane
    
    Returns:
        np.ndarray: Array of points on the Bézier curve.
    """
    p0 = x_curr
    p1 = (x_curr[0], 0.45)
    
    p3 = tuple(np.add(x_curr, (step_length, 0)))
    p2 = (p3[0], 0.50)


    t_values = np.linspace(0, 1, array_size)
    points = np.zeros((array_size, 4))

    x_stance_leg = np.linspace(-x_curr[0], -x_curr[0] - step_length, array_size)
    points[:, 2] = x_stance_leg
    points[:, 3] = [x_curr[1]] * array_size

    for i, t in enumerate(t_values):
        x = (1 - t)**3 * p0[0] + 3 * (1 - t)**2 * t * p1[0] + 3 * (1 - t) * t**2 * p2[0] + t**3 * p3[0]
        z = (1 - t)**3 * p0[1] + 3 * (1 - t)**2 * t * p1[1] + 3 * (1 - t) * t**2 * p2[1] + t**3 * p3[1]
        points[i, :2] = [x, z]
    
    print(points.shape)

    # Plot the points
    plt.figure()
    plt.plot(points[:, 0], points[:, 1])
    plt.plot(points[:, 2], points[:, 3])
    plt.title('Cubic Bézier Curve')
    plt.xlabel('x')
    plt.ylabel('z')
    plt.grid(True)
    plt.show()

    return points

def package_bezier(step_length, array_size):

    xzpositions_first_step = np.asfortranarray([[0.0, (0.02/0.1)*(step_length/2), (0.0533/0.1)*(step_length/2), (step_length/2)], [0.0, 0.4, 0.2, 0.0]])
    curvexz_first_step = bezier.Curve(xzpositions_first_step, degree=3)
    number_of_time_points_first_step = np.linspace(0, 1.0, int(array_size/2))
    points_first_step= curvexz_first_step.evaluate_multi(number_of_time_points_first_step)
    x_swing_first_step = points_first_step[0,:]
    z_swing_first_step = points_first_step[1,:]
    x_stance_first_step = np.linspace(0, 0 - (step_length/2), int(array_size/2))
    z_stance_first_step = [0]*int(array_size/2)
    final_points_first_step = np.column_stack((x_swing_first_step, z_swing_first_step, x_stance_first_step, z_stance_first_step))
    plt.plot(x_swing_first_step, z_swing_first_step)
    plt.plot(x_stance_first_step, z_stance_first_step, color="orange")
    plt.show()
    # print(final_points.shape)

    xzpositions_complete_step = np.asfortranarray([[0.0, (0.02/0.1)*step_length, (0.0533/0.1)*step_length, step_length], [0.0, 0.4, 0.2, 0.0]])
    curvexz_complete_step = bezier.Curve(xzpositions_complete_step, degree=3)
    number_of_time_points_complete_step = np.linspace(0, 1.0, array_size)
    points_complete_step= curvexz_complete_step.evaluate_multi(number_of_time_points_complete_step)
    # x_swing_complete_step = points_complete_step[0,:] - (step_length/2)
    x_swing_complete_step = points_complete_step[0,:]
    z_swing_complete_step = points_complete_step[1,:]
    # x_stance_complete_step = np.linspace(0+(step_length/2), 0 - (step_length/2), array_size)
    x_stance_complete_step = np.linspace(0+step_length, 0, array_size)
    z_stance_complete_step = [0]*array_size
    final_points_complete_step = np.column_stack((x_swing_complete_step, z_swing_complete_step, x_stance_complete_step, z_stance_complete_step))
    plt.plot(x_swing_complete_step, z_swing_complete_step)
    plt.plot(x_stance_complete_step, z_stance_complete_step, color="orange")
    plt.show()

    return final_points_first_step, final_points_complete_step

#TODO: change x_curr input to actually be foot input from state estimator. 

# first_step_large = cubic_bezier(20, 0.20, (0, -0.78))

# Generate the normal gait large array
# normal_gait_large = cubic_bezier(40, 0.40, (-.1, -0.78))

# Generate the first step large array
# first_step_small = cubic_bezier(20, 0.10, (0, 0))

# Generate the normal gait large array
# normal_gait_small = cubic_bezier(40, 0.20, (0, 0))

# Save the arrays to separate CSV files
# column_names = ['x', 'z']
# np.savetxt('src/march_gait_planning/m9_gait_files/cartesian/first_step_large.csv', first_step_large, delimiter=',')
# np.savetxt('src/march_gait_planning/m9_gait_files/cartesian/normal_gait_large.csv', normal_gait_large, delimiter=',')

# np.savetxt('ros2/src/march_gait_planning/m9_gait_files/cartesian/first_step_small.csv', first_step_small, delimiter=',')
# np.savetxt('ros2/src/march_gait_planning/m9_gait_files/cartesian/normal_gait_small.csv', normal_gait_small, delimiter=',')
    
large_gait_first_step, large_gait_complete_step = package_bezier(0.6, 80)
np.savetxt('ros2/src/march_gait_planning/m9_gait_files/cartesian/first_step_large.csv', large_gait_first_step, delimiter=',')
np.savetxt('ros2/src/march_gait_planning/m9_gait_files/cartesian/normal_gait_large.csv', large_gait_complete_step, delimiter=',')
small_gait_first_step, small_gait_complete_step = package_bezier(0.2, 80)
# np.savetxt('ros2/src/march_gait_planning/m9_gait_files/cartesian/first_step_small.csv', small_gait_first_step, delimiter=',')
# np.savetxt('ros2/src/march_gait_planning/m9_gait_files/cartesian/normal_gait_small.csv', small_gait_complete_step, delimiter=',')

def interpolate(step_size, small_gait_complete_step, large_gait_complete_step, array_size):
    z_swing = []
    x_swing = np.linspace(0, step_size/2, array_size)
    for i in range(array_size): 
        try: 
            z = small_gait_complete_step[i][1] + (x_swing[i]-small_gait_complete_step[i][0]) * ((large_gait_complete_step[i][1]-small_gait_complete_step[i][1])/(large_gait_complete_step[i][0]-small_gait_complete_step[i][0]))
            z_swing.append(z)
        except RuntimeWarning: 
            z_swing.append(0.0)
    x_stance = np.linspace(0.0, -step_size/2, array_size)
    z_stance = np.linspace(0.0, 0.0, array_size)
    result = np.column_stack((x_swing, z_swing, x_stance, z_stance))
    plt.plot(result[:,0], result[:,1])
    plt.plot(result[:,2], result[:,3])
    plt.plot(small_gait_complete_step[:,0], small_gait_complete_step[:,1])
    plt.plot(large_gait_complete_step[:,0], large_gait_complete_step[:,1])
    plt.show()
    return result 

res = interpolate(0.3, small_gait_first_step, large_gait_first_step, 20)

# plt.plot(large_gait_complete_step[:,0], large_gait_complete_step[:,1])
# plt.plot(small_gait_complete_step[:,0], small_gait_complete_step[:,1])
# plt.show()