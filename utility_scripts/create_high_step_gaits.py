import numpy as np
import matplotlib.pyplot as plt
import os
import bezier 

def high_step(step_height, array_size):
    step_length = 0.3

    xzpositions_first_step = np.asfortranarray([[0.0, (0.02/0.1)*(step_length/2), (0.0533/0.1)*(step_length/2), (step_length/2)], [0.0, 0.4, 0.2 + step_height, 0.0 + step_height]])
    curvexz_first_step = bezier.Curve(xzpositions_first_step, degree=3)
    number_of_time_points_first_step = np.linspace(0, 1.0, int(array_size/2))
    points_first_step= curvexz_first_step.evaluate_multi(number_of_time_points_first_step)
    x_swing_first_step = points_first_step[0,:]
    z_swing_first_step = points_first_step[1,:]
    x_stance_first_step = np.linspace(0, 0 - (step_length/2), int(array_size/2))
    # z_stance_first_step = compensation_for_circle(int(array_size/2), step_length/2)
    z_stance_first_step = [0]*int(array_size/2)
    # z_swing_first_step = z_swing_first_step + z_stance_first_step
    final_points_first_step = np.column_stack((x_swing_first_step, z_swing_first_step, x_stance_first_step, z_stance_first_step))
    plt.plot(x_swing_first_step, z_swing_first_step)
    plt.plot(x_stance_first_step, z_stance_first_step, color="orange")
    plt.show()

    return final_points_first_step

high_step1 = high_step(0.26, 200)
np.savetxt('ros2/src/march_gait_planning/m9_gait_files/cartesian/high_step1.csv', high_step1, delimiter=',')