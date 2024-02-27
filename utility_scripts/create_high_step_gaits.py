import numpy as np
import matplotlib.pyplot as plt
import os
import bezier 
from scipy.interpolate import interp1d

def high_step(step_height, array_size):

    step_length = 0.3

    # Part 1 
    swing_leg_high_step = np.asfortranarray([[0.0, (0.02/0.1)*(step_length/2), (0.0533/0.1)*(step_length/2), (step_length/2)], [0.0, 0.4, 0.2 + step_height, 0.0 + step_height]])
    curve_high_step = bezier.Curve(swing_leg_high_step, degree=3)
    number_of_time_points_high_step = np.linspace(0, 1.0, int(array_size/2))
    points_high_step= curve_high_step.evaluate_multi(number_of_time_points_high_step)

    x_swing_high_step = points_high_step[0,:]
    z_swing_high_step = points_high_step[1,:]
    x_stance_high_step = np.linspace(0, 0 - (step_length/2), int(array_size/2))
    z_stance_high_step = [0]*int(array_size/2)

    # Part 2, step close 

    begin_coordinate_swing = [x_swing_high_step[-1], z_swing_high_step[-1]]
    end_coordinate_swing = [begin_coordinate_swing[0], 0.0]
    begin_coordinate_stance = [x_stance_high_step[-1], z_stance_high_step[-1]] #dataset[-1, 2:-1]

    close_step_bezier = np.asfortranarray([[0.0, (0.02/0.1)*(step_length/2), (0.0533/0.1)*(step_length/2), (step_length/2)], [0.0, 0.4, 0.2, 0.0]])
    curve_close_step  = bezier.Curve(close_step_bezier, degree=3)
    number_of_time_points_close_step = np.linspace(0, 1.0, int(array_size/2))
    points_close_step = curve_close_step.evaluate_multi(number_of_time_points_close_step)
    
    x_stance_close_step = points_close_step[0,:] + begin_coordinate_stance[0]
    z_stance_close_step = points_close_step[1,:]
    x_swing_close_step = np.linspace(begin_coordinate_swing[0], end_coordinate_swing[0], int(array_size/2))
    z_swing_close_step = np.linspace(begin_coordinate_swing[1], end_coordinate_swing[1], int(array_size/2))

    # Combine everything

    x_swing_high_step = np.append(x_swing_high_step, x_swing_close_step)
    z_swing_high_step = np.append(z_swing_high_step, z_swing_close_step)
    x_stance_high_step = np.append(x_stance_high_step, x_stance_close_step)
    z_stance_high_step = np.append(z_stance_high_step, z_stance_close_step)

    final_points_high_step = np.column_stack((x_swing_high_step, z_swing_high_step, x_stance_high_step, z_stance_high_step))

    plt.plot(x_swing_high_step, z_swing_high_step)
    plt.plot(x_stance_high_step, z_stance_high_step, color="orange")
    plt.show()

    return final_points_high_step

def interpolate_linear(dataset, array_size):
    begin_coordinate_swing = dataset[-1, 0:2]
    end_coordinate_swing = [begin_coordinate_swing[0], 0.0]

    begin_coordinate_stance = dataset[-1, 2:-1]
    end_coordinate_stance = [0.0, 0.0]
    step_length = abs(begin_coordinate_stance[0]) - end_coordinate_stance[0]
    step_height = 10
    stance_bezier = np.asfortranarray([[0.0, (0.02/0.1)*(step_length), (0.0533/0.1)*(step_length), (step_length)], [0.0, 0.4, 0.2, 0.0]])

    curve_stance  = bezier.Curve(stance_bezier, degree=3)
    number_of_time_points_first_step = np.linspace(0, 1.0, int(array_size/2))
    points_first_step= curve_stance.evaluate_multi(number_of_time_points_first_step)
    x_stance = points_first_step[0,:] + begin_coordinate_stance[0]
    z_stance = points_first_step[1,:]
    # x_vals = [begin_coordinate_swing[0], end_coordinate_swing[0]]
    # z_vals = [begin_coordinate_swing[1], end_coordinate_swing[1]]
    time_steps = np.linspace(begin_coordinate_swing[0], end_coordinate_swing[0], int(array_size/2))
    interpolated_vals_swing = np.linspace(begin_coordinate_swing[1], end_coordinate_swing[1], int(array_size/2))
    # interpolated_vals = interp1d(x_vals, z_vals)
    # plt.plot(time_steps, interpolated_vals(time_steps))
    plt.plot(time_steps, interpolated_vals_swing)
    plt.plot(x_stance, z_stance)
    plt.show()

high_step1 = high_step(0.26, 200)
# interpolate_linear(high_step1, 200)
np.savetxt('ros2/src/march_gait_planning/m9_gait_files/cartesian/high_step1.csv', high_step1, delimiter=',')