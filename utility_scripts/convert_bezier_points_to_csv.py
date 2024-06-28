import numpy as np
import matplotlib.pyplot as plt
import bezier 


def create_bezier_csv(points, array_size, gait_type):
    step_length = points.max(axis=0)[0]

    first_step_points = points.copy()
    first_step_points[:, 0] = first_step_points[:, 0]/2
    curvexz_first_step = bezier.Curve(first_step_points.T, degree=3)
    number_of_time_points_first_step = np.linspace(0, 1.0, int(array_size/2))
    points_first_step= curvexz_first_step.evaluate_multi(number_of_time_points_first_step)
    x_swing_first_step = points_first_step[0,:]
    z_swing_first_step = points_first_step[1,:]
    x_stance_first_step = np.linspace(0, 0 - (step_length/2), int(array_size/2))
    # z_stance_first_step = compensation_for_circle(int(array_size/2), step_length/2)
    z_stance_first_step = [0]*int(array_size/2)
    # z_swing_first_step = z_swing_first_step + z_stance_first_step
    final_points_first_step = np.column_stack((x_swing_first_step, z_swing_first_step, x_stance_first_step, z_stance_first_step))

    curvexz_complete_step = bezier.Curve(points.T, degree=3)
    print(points.T)
    number_of_time_points_complete_step = np.linspace(0, 1.0, array_size)
    points_complete_step= curvexz_complete_step.evaluate_multi(number_of_time_points_complete_step)
    x_swing_complete_step = points_complete_step[0,:] - (step_length/2)
    z_swing_complete_step = points_complete_step[1,:]
    x_stance_complete_step = np.linspace(0+(step_length/2), 0 - (step_length/2), array_size)
    # z_stance_complete_step = compensation_for_circle(array_size, step_length)
    z_stance_complete_step = [0]*array_size
    # z_swing_complete_step = z_swing_complete_step + z_stance_complete_step
    final_points_complete_step = np.column_stack((x_swing_complete_step, z_swing_complete_step, x_stance_complete_step, z_stance_complete_step))

    step_close_points = points.copy()
    step_close_points[:, 0] = step_close_points[:, 0]/-2
    curvexz_step_close = bezier.Curve(step_close_points.T, degree=3)
    number_of_time_points_step_close = np.linspace(0, 1.0, int(array_size/2))
    points_step_close= curvexz_step_close.evaluate_multi(number_of_time_points_step_close)
    x_swing_step_close = points_step_close[0,:]
    z_swing_step_close = points_step_close[1,:]
    x_stance_step_close = np.linspace(0, 0 - (-step_length/2), int(array_size/2))
    # z_stance_first_step = compensation_for_circle(int(array_size/2), step_length/2)
    z_stance_step_close = [0]*int(array_size/2)
    # z_swing_first_step = z_swing_first_step + z_stance_first_step
    final_points_step_close = np.column_stack((x_swing_step_close, z_swing_step_close, x_stance_step_close, z_stance_step_close))
    final_points_step_close = np.flip(final_points_step_close, axis=0)

    if gait_type == "large_gait":
        np.savetxt('ros2/src/march_gait_planning/m9_gait_files/cartesian/first_step_large.csv', final_points_first_step, delimiter=',')
        np.savetxt('ros2/src/march_gait_planning/m9_gait_files/cartesian/normal_gait_large.csv', final_points_complete_step, delimiter=',')
        np.savetxt('ros2/src/march_gait_planning/m9_gait_files/cartesian/large_step_close.csv', final_points_step_close, delimiter=',')
    elif gait_type == "small_gait":
        np.savetxt('ros2/src/march_gait_planning/m9_gait_files/cartesian/first_step_small.csv', final_points_first_step, delimiter=',')
        np.savetxt('ros2/src/march_gait_planning/m9_gait_files/cartesian/normal_gait_small.csv', final_points_complete_step, delimiter=',')
        np.savetxt('ros2/src/march_gait_planning/m9_gait_files/cartesian/small_step_close.csv', final_points_step_close, delimiter=',')

def stair_gaits(set_of_points, array_size):
    first_step_points = np.array(set_of_points[0]["first_step"])
    full_step_points = np.array(set_of_points[1]["full_step"])
    step_close_points = np.array(set_of_points[2]["step_close"])
    number_of_time_points = np.linspace(0, 1.0, int(array_size/2))

    # Part 1: swing leg goes up one step, stance leg stays on the ground
    first_step_curve = bezier.Curve(first_step_points.T, degree=3)
    points= first_step_curve.evaluate_multi(number_of_time_points)

    x_swing_stairs = points[0,:]
    z_swing_stairs = points[1,:]
    x_stance_stairs = np.linspace(first_step_points[0][0], -first_step_points[3][0], int(array_size/2))
    z_stance_stairs = [0]*int(array_size/2)

    first_points_stairs = np.column_stack((x_swing_stairs, z_swing_stairs, x_stance_stairs, z_stance_stairs))

    # Part 2: swing leg goes down two steps, stance leg stays still
    full_step_curve = bezier.Curve(full_step_points.T, degree=3)
    points= full_step_curve.evaluate_multi(number_of_time_points)

    x_swing_stair = points[0,:]
    z_swing_stair = points[1,:]
    x_stance_stair = np.linspace(full_step_points[3][0], full_step_points[0][0], int(array_size/2))
    z_stance_stair = np.linspace(full_step_points[3][1], full_step_points[0][1], int(array_size/2))

    # Part 3: swing leg closes to stance leg (top of the stairs)
    step_close_curve = bezier.Curve(step_close_points.T, degree=3)
    points = step_close_curve.evaluate_multi(number_of_time_points)

    x_swing_last_step = points[0,:]
    z_swing_last_step = points[1,:]
    x_stance_last_step = np.linspace(-step_close_points[0][0], step_close_points[3][0] , int(array_size/2))
    z_stance_last_step = np.linspace(full_step_points[3][1], 0, int(array_size/2))

    second_step_points_stairs = np.column_stack((x_stance_stair, z_stance_stair, x_swing_stair, z_swing_stair))

    third_step_points_stairs = np.column_stack((x_swing_stair, z_swing_stair, x_stance_stair, z_stance_stair))

    forth_step_points_stairs = np.column_stack((x_stance_stair, z_stance_stair, x_swing_stair, z_swing_stair))

    final_step_points_stairs = np.column_stack((x_swing_last_step, z_swing_last_step, x_stance_last_step, z_stance_last_step))



    final_points_stairs_up = np.concatenate((first_points_stairs, second_step_points_stairs, third_step_points_stairs, forth_step_points_stairs, final_step_points_stairs), axis=0)


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