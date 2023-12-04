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
    p1 = (x_curr[0], -0.82)
    
    p3 = tuple(np.add(x_curr, (step_length, 0)))
    print(p3)
    p2 = (p3[0], -0.89)


    t_values = np.linspace(0, 1, array_size)
    points = np.zeros((array_size, 2))

    for i, t in enumerate(t_values):
        x = (1 - t)**3 * p0[0] + 3 * (1 - t)**2 * t * p1[0] + 3 * (1 - t) * t**2 * p2[0] + t**3 * p3[0]
        z = (1 - t)**3 * p0[1] + 3 * (1 - t)**2 * t * p1[1] + 3 * (1 - t) * t**2 * p2[1] + t**3 * p3[1]
        points[i] = [x, z]

    # # Plot the points
    # plt.figure()
    # plt.plot(points[:, 0], points[:, 1])
    # plt.title('Cubic Bézier Curve')
    # plt.xlabel('x')
    # plt.ylabel('z')
    # plt.grid(True)
    # plt.show()

    return points


# Generate the first step array
first_step = cubic_bezier(1000, 0.10, (0, -0.95))

# Generate the normal gait array
normal_gait = cubic_bezier(1000, 0.20, (-.1, -0.95))

# Save the arrays to separate CSV files
np.savetxt('ros2/src/march_gait_planning/m9_gait_files/first_step.csv', first_step, delimiter=',')
np.savetxt('ros2/src/march_gait_planning/m9_gait_files/normal_gait.csv', normal_gait, delimiter=',')