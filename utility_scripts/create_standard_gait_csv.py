import numpy as np
import matplotlib.pyplot as plt
import os


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
    p1 = (x_curr[0], -0.70)
    
    p3 = tuple(np.add(x_curr, (step_length, 0)))
    print(p3)
    p2 = (p3[0], -0.75)


    t_values = np.linspace(0, 1, array_size)
    points = np.zeros((array_size, 4))

    x_stance_leg = np.linspace(-x_curr[0], -x_curr[0] - step_length, array_size)
    points[:, 2] = x_stance_leg
    points[:, 3] = [x_curr[1]] * array_size

    for i, t in enumerate(t_values):
        x = (1 - t)**3 * p0[0] + 3 * (1 - t)**2 * t * p1[0] + 3 * (1 - t) * t**2 * p2[0] + t**3 * p3[0]
        z = (1 - t)**3 * p0[1] + 3 * (1 - t)**2 * t * p1[1] + 3 * (1 - t) * t**2 * p2[1] + t**3 * p3[1]
        points[i, :2] = [x, z]

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


# Generate the first step array
first_step = cubic_bezier(20, 0.10, (0, -0.78))

# Generate the normal gait array
normal_gait = cubic_bezier(40, 0.20, (-.1, -0.78))

# Save the arrays to separate CSV files
column_names = ['x', 'z']
np.savetxt('src/march_gait_planning/m9_gait_files/cartesian/first_step.csv', first_step, delimiter=',')
np.savetxt('src/march_gait_planning/m9_gait_files/cartesian/normal_gait.csv', normal_gait, delimiter=',')
