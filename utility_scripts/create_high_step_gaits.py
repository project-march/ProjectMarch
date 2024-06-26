import numpy as np
import matplotlib.pyplot as plt
import os
import bezier 
from scipy.interpolate import interp1d

def high_step_up(step_height, array_size, step_length):

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

    begin_coordinate_swing_closte_step = [x_swing_high_step[-1], z_swing_high_step[-1]]
    # end_coordinate_swing_close_step = [begin_coordinate_swing_closte_step[0], 0.0]
    end_coordinate_swing_close_step = [0.0, 0.0]
    begin_coordinate_stance_close_step = [x_stance_high_step[-1], z_stance_high_step[-1]] #dataset[-1, 2:-1]

    close_step_bezier = np.asfortranarray([[0.0, (0.02/0.1)*(step_length/2), (0.0533/0.1)*(step_length/2), (step_length/2)], [0.0, 0.4, 0.2, 0.0]])
    curve_close_step  = bezier.Curve(close_step_bezier, degree=3)
    number_of_time_points_close_step = np.linspace(0, 1.0, int(array_size/2))
    points_close_step = curve_close_step.evaluate_multi(number_of_time_points_close_step)
    
    x_stance_close_step = points_close_step[0,:] + begin_coordinate_stance_close_step[0]
    z_stance_close_step = points_close_step[1,:]
    x_swing_close_step = np.linspace(begin_coordinate_swing_closte_step[0], end_coordinate_swing_close_step[0], int(array_size/2))
    z_swing_close_step = np.linspace(begin_coordinate_swing_closte_step[1], end_coordinate_swing_close_step[1], int(array_size/2))

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

def high_step_down(dataset):

    # x_stance = np.flip(dataset[:, 0], 0)
    # z_stance = np.flip(dataset[:, 1], 0)
    # x_swing = np.flip(dataset[:, 2], 0)
    # z_swing = np.flip(dataset[:, 3], 0)
    x_stance = dataset[:,0][::-1]*-1
    z_stance = dataset[:,1][::-1]
    x_swing = dataset[:,2][::-1]*-1
    z_swing = dataset[:,3][::-1]

    down_step = np.column_stack((x_swing, z_swing, x_stance, z_stance))

    plt.plot(x_swing, z_swing)
    plt.plot(x_stance, z_stance, color="orange")
    plt.show()

    return down_step
    

high_step1_up = high_step_up(0.26, 400, 0.3)
high_step1_down = high_step_down(high_step1_up)
np.savetxt('ros2/src/march_gait_planning/m9_gait_files/cartesian/high_step1.csv', high_step1_up, delimiter=',')
np.savetxt('ros2/src/march_gait_planning/m9_gait_files/cartesian/high_step1_down.csv', high_step1_down, delimiter=',')
high_step2_up = high_step_up(0.22, 400, 0.3)
high_step2_down = high_step_down(high_step2_up)
high_step3_up = high_step_up(0.18, 400, 0.3)
high_step3_down = high_step_down(high_step3_up)
np.savetxt('ros2/src/march_gait_planning/m9_gait_files/cartesian/high_step2.csv', high_step2_up, delimiter=',')
np.savetxt('ros2/src/march_gait_planning/m9_gait_files/cartesian/high_step2_down.csv', high_step2_down, delimiter=',')
np.savetxt('ros2/src/march_gait_planning/m9_gait_files/cartesian/high_step3.csv', high_step3_up, delimiter=',')
np.savetxt('ros2/src/march_gait_planning/m9_gait_files/cartesian/high_step3_down.csv', high_step3_down, delimiter=',')