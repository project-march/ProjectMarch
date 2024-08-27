import numpy as np
import matplotlib.pyplot as plt
import os
import bezier 
from scipy.interpolate import interp1d

def ascend_stairs(step_height, array_size, step_length):

    # Part 1: swing leg goes up one step, stance leg stays on the ground
    swing_leg = np.asfortranarray([[0.0, (0.02/0.1)*(step_length/2), (0.0533/0.1)*(step_length/2), (step_length/2)], [0.0, 0.4, 0.2 + step_height, 0.0 + step_height]])
    curve = bezier.Curve(swing_leg, degree=3)
    number_of_time_points = np.linspace(0, 1.0, int(array_size/2))
    points= curve.evaluate_multi(number_of_time_points)

    x_swing_stairs = points[0,:]
    z_swing_stairs = points[1,:]
    x_stance_stairs = np.linspace(0, 0 - (step_length/2), int(array_size/2))
    z_stance_stairs = [0]*int(array_size/2)

    first_points_stairs = np.column_stack((x_swing_stairs, z_swing_stairs, x_stance_stairs, z_stance_stairs))

    # Part 2: swing leg goes down two steps, stance leg stays still
    swing_leg_stair = np.asfortranarray([[-step_length/2, (0.02/0.1)*(step_length/2), (0.0533/0.1)*(step_length/2), (step_length/2)], [0.0, 0.4, 0.2 + step_height, step_height]])
    curve_stair = bezier.Curve(swing_leg_stair, degree=3)
    number_of_time_points_stair = np.linspace(0, 1.0, int(array_size/2))
    points_stair= curve_stair.evaluate_multi(number_of_time_points_stair)

    x_swing_stair = points_stair[0,:]
    z_swing_stair = points_stair[1,:]
    x_stance_stair = np.linspace((step_length/2), 0 - (step_length/2), int(array_size/2))
    z_stance_stair = np.linspace(step_height, 0, int(array_size/2))

    # Part 3: swing leg closes to stance leg (top of the stairs)
    swing_leg_last_step = np.asfortranarray([[-step_length/2, -(0.02/0.1)*(step_length/2), 0, 0], [0.0, 0.4, 0.2, 0]])
    curve_last_step = bezier.Curve(swing_leg_last_step, degree=3)
    number_of_time_points_last_step = np.linspace(0, 1.0, int(array_size/2))
    points_last_step = curve_last_step.evaluate_multi(number_of_time_points_last_step)

    x_swing_last_step = points_last_step[0,:]
    z_swing_last_step = points_last_step[1,:]
    x_stance_last_step = np.linspace((step_length/2), 0 , int(array_size/2))
    z_stance_last_step = np.linspace(step_height, 0, int(array_size/2))

    second_step_points_stairs = np.column_stack((x_stance_stair, z_stance_stair, x_swing_stair, z_swing_stair))

    third_step_points_stairs = np.column_stack((x_swing_stair, z_swing_stair, x_stance_stair, z_stance_stair))

    forth_step_points_stairs = np.column_stack((x_stance_stair, z_stance_stair, x_swing_stair, z_swing_stair))

    final_step_points_stairs = np.column_stack((x_swing_last_step, z_swing_last_step, x_stance_last_step, z_stance_last_step))



    final_points_stairs_up = np.concatenate((first_points_stairs, second_step_points_stairs, third_step_points_stairs, forth_step_points_stairs, final_step_points_stairs), axis=0)

    print(final_points_stairs_up.shape)

    plt.plot(final_points_stairs_up[:,0], final_points_stairs_up[:,1])
    plt.plot(final_points_stairs_up[:,2], final_points_stairs_up[:,3], color="orange")
    plt.show()

    return final_points_stairs_up

def descend_stairs(data_up): 
     
    x_stance = data_up[:,0][::-1]*-1
    z_stance = data_up[:,1][::-1]
    x_swing = data_up[:,2][::-1]*-1
    z_swing = data_up[:,3][::-1]

    final_points_stairs_down = np.column_stack((x_swing, z_swing, x_stance, z_stance))

    plt.plot(x_swing, z_swing)
    plt.plot(x_stance, z_stance, color="orange")
    plt.show()
    return final_points_stairs_down

stairs_up = ascend_stairs(0.171, 1000, 0.28)
stairs_down = descend_stairs(stairs_up)
np.savetxt('ros2/src/march_gait_planning/m9_gait_files/cartesian/ascend_test.csv', stairs_up, delimiter=',')
np.savetxt('ros2/src/march_gait_planning/m9_gait_files/cartesian/descend_test.csv', stairs_down, delimiter=',')

