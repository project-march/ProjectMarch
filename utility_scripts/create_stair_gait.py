import numpy as np
import matplotlib.pyplot as plt
import os
import bezier 
from scipy.interpolate import interp1d

def ascend_stairs(step_height, array_size, step_length):

    # Part 1: swing leg goes up one step, stance leg stays on the ground
    swing_leg_high_step = np.asfortranarray([[0.0, (0.02/0.1)*(step_length/2), (0.0533/0.1)*(step_length/2), (step_length/2)], [0.0, 0.4, 0.2 + step_height, 0.0 + step_height]])
    curve_high_step = bezier.Curve(swing_leg_high_step, degree=3)
    number_of_time_points_high_step = np.linspace(0, 1.0, int(array_size/2))
    points_high_step= curve_high_step.evaluate_multi(number_of_time_points_high_step)

    x_swing_high_step = points_high_step[0,:]
    z_swing_high_step = points_high_step[1,:]
    x_stance_high_step = np.linspace(0, 0 - (step_length/2), int(array_size/2))
    z_stance_high_step = [0]*int(array_size/2)

    first_points_stairs = np.column_stack((x_swing_high_step, z_swing_high_step, x_stance_high_step, z_stance_high_step))

    # Part 2: swing leg goes down two step, stance leg stays still
    swing_leg_stair = np.asfortranarray([[-step_length/2, (0.02/0.1)*(step_length/2), (0.0533/0.1)*(step_length/2), (step_length/2)], [0.0, 0.4, 0.2 + step_height, step_height]])
    print(swing_leg_stair)
    curve_stair = bezier.Curve(swing_leg_stair, degree=3)
    number_of_time_points_stair = np.linspace(0, 1.0, int(array_size/2))
    points_stair= curve_stair.evaluate_multi(number_of_time_points_stair)

    x_swing_stair = points_stair[0,:]
    z_swing_stair = points_stair[1,:]
    x_stance_stair = np.linspace((step_length/2), 0 - (step_length/2), int(array_size/2))
    z_stance_stair = np.linspace(step_height, 0, int(array_size/2))

    final_points_stairs = np.column_stack((x_stance_stair, z_stance_stair, x_swing_stair, z_swing_stair))

    final_points_high_step = np.concatenate((first_points_stairs, final_points_stairs), axis=0)

    print(final_points_high_step.shape)

    plt.plot(final_points_high_step[:,0], final_points_high_step[:,1])
    plt.plot(final_points_high_step[:,2], final_points_high_step[:,3], color="orange")
    plt.show()

    return final_points_high_step

high_step1_up = ascend_stairs(0.171, 200, 0.28)
np.savetxt('ros2/src/march_gait_planning/m9_gait_files/cartesian/test.csv', high_step1_up, delimiter=',')