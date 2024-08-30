import numpy as np
import matplotlib.pyplot as plt
import bezier 
from scipy.interpolate import interp1d

LEG_LENGTH = 0.912
HIGH_LEVEL_FREQUENCY = 50

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

def create_bezier_csv(points, array_size, gait_type):
    pause_time = int(array_size/6)
    step_length = points.max(axis=0)[0]
    ending_angle = np.arcsin((step_length/2)/LEG_LENGTH)

    # ------------------------------------------------- FIRST STEP ----------------------------------------------------------------------------------
    phi_first_step = np.linspace(0, ending_angle, int(array_size))

    # The stance leg follows a circle with radius LEG_LENGTH with center at (0, LEG_LENGTH). Sine and cosine are swapped from a usual circle, since we want the circle to start at the bottom and go backwards
    x_stance_first_step = -LEG_LENGTH*np.sin(phi_first_step)
    z_stance_first_step = LEG_LENGTH - LEG_LENGTH*np.cos(phi_first_step)

    x_stance_first_step = np.append(x_stance_first_step, x_stance_first_step[-1]*np.ones(pause_time))
    z_stance_first_step = np.append(z_stance_first_step, z_stance_first_step[-1]*np.ones(pause_time))

    vertical_offset = z_stance_first_step[-1]

    first_step_points = points.copy()
    first_step_points[:, 0] = first_step_points[:, 0]/2
    first_step_points[1:, 1] = first_step_points[1:, 1] + vertical_offset

    curvexz_first_step = bezier.Curve(first_step_points.T, degree=3)
    number_of_time_points_first_step = np.linspace(0, 1.0, int(array_size))
    points_first_step= curvexz_first_step.evaluate_multi(number_of_time_points_first_step)
    x_swing_first_step = points_first_step[0,:]
    z_swing_first_step = points_first_step[1,:]

    x_swing_first_step, z_swing_first_step = make_evenly_spaced_points(x_swing_first_step, z_swing_first_step, int(array_size))

    x_swing_first_step = np.append(x_swing_first_step, x_swing_first_step[-1]*np.ones(pause_time))
    z_swing_first_step = np.append(z_swing_first_step, z_swing_first_step[-1]*np.ones(pause_time))

    final_points_first_step = np.column_stack((x_swing_first_step, z_swing_first_step, x_stance_first_step, z_stance_first_step))

    # plt.scatter(x_swing_first_step, z_swing_first_step)
    # plt.plot(x_stance_first_step, z_stance_first_step, color="orange")
    # plt.title("First Step")
    # plt.show()

    # ------------------------------------------------- FULL STEP ----------------------------------------------------------------------------------
    phi_full_step = np.linspace(-ending_angle, ending_angle, int(array_size))

    x_stance_complete_step = -LEG_LENGTH*np.sin(phi_full_step)
    z_stance_complete_step = LEG_LENGTH - LEG_LENGTH*np.cos(phi_full_step)

    x_stance_complete_step = np.append(x_stance_complete_step, x_stance_complete_step[-1]*np.ones(pause_time))
    z_stance_complete_step = np.append(z_stance_complete_step, z_stance_complete_step[-1]*np.ones(pause_time))

    full_step_points = points.copy()
    full_step_points[:, 1] = full_step_points[:, 1] + vertical_offset

    curvexz_complete_step = bezier.Curve(full_step_points.T, degree=3)
    number_of_time_points_complete_step = np.linspace(0, 1.0, array_size)
    points_complete_step= curvexz_complete_step.evaluate_multi(number_of_time_points_complete_step)
    x_swing_complete_step = points_complete_step[0,:] - (step_length/2)
    z_swing_complete_step = points_complete_step[1,:]
    
    x_swing_complete_step, z_swing_complete_step = make_evenly_spaced_points(x_swing_complete_step, z_swing_complete_step, array_size)
    
    x_swing_complete_step = np.append(x_swing_complete_step, x_swing_complete_step[-1]*np.ones(pause_time))
    z_swing_complete_step = np.append(z_swing_complete_step, z_swing_complete_step[-1]*np.ones(pause_time))

    final_points_complete_step = np.column_stack((x_swing_complete_step, z_swing_complete_step, x_stance_complete_step, z_stance_complete_step))

    # plt.plot(x_swing_complete_step, z_swing_complete_step)
    # plt.plot(x_stance_complete_step, z_stance_complete_step, color="orange")
    # plt.title("Full Step")
    # plt.show()

    # ------------------------------------------------- STEP CLOSE ----------------------------------------------------------------------------------
    phi_step_close = np.linspace(0, -ending_angle, int(array_size))

    x_stance_step_close = -LEG_LENGTH*np.sin(phi_step_close)
    z_stance_step_close = LEG_LENGTH - LEG_LENGTH*np.cos(phi_step_close)

    # x_stance_step_close = np.append(x_stance_step_close, x_stance_step_close[-1]*np.ones(pause_time))
    # z_stance_step_close = np.append(z_stance_step_close, z_stance_step_close[-1]*np.ones(pause_time))

    step_close_points = points.copy()
    step_close_points[:, 0] = step_close_points[:, 0]/-2
    step_close_points[1:, 1] = step_close_points[1:, 1] + vertical_offset
    step_close_points[2, 0] = step_close_points[2, 0] * 0.3
    curvexz_step_close = bezier.Curve(step_close_points.T, degree=3)
    number_of_time_points_step_close = np.linspace(0, 1.0, int(array_size))
    points_step_close= curvexz_step_close.evaluate_multi(number_of_time_points_step_close)
    x_swing_step_close = points_step_close[0,:]
    z_swing_step_close = points_step_close[1,:]

    x_swing_step_close, z_swing_step_close = make_evenly_spaced_points(x_swing_step_close, z_swing_step_close, array_size)

    # x_swing_step_close = np.append(x_swing_step_close, x_swing_step_close[-1]*np.ones(pause_time))
    # z_swing_step_close = np.append(z_swing_step_close, z_swing_step_close[-1]*np.ones(pause_time))

    final_points_step_close = np.column_stack((x_swing_step_close, z_swing_step_close, x_stance_step_close, z_stance_step_close))
    final_points_step_close = np.flip(final_points_step_close, axis=0)

    plt.plot(x_swing_step_close, z_swing_step_close)
    plt.plot(x_stance_step_close, z_stance_step_close, color="orange")
    plt.title("Step Close")
    plt.show()

    if gait_type == "large_gait":
        np.savetxt('ros2/src/march_gait_planning/m9_gait_files/cartesian/first_step_large.csv', final_points_first_step, delimiter=',')
        np.savetxt('ros2/src/march_gait_planning/m9_gait_files/cartesian/normal_gait_large.csv', final_points_complete_step, delimiter=',')
        np.savetxt('ros2/src/march_gait_planning/m9_gait_files/cartesian/large_step_close.csv', final_points_step_close, delimiter=',')
    elif gait_type == "small_gait":
        np.savetxt('ros2/src/march_gait_planning/m9_gait_files/cartesian/first_step_small.csv', final_points_first_step, delimiter=',')
        np.savetxt('ros2/src/march_gait_planning/m9_gait_files/cartesian/normal_gait_small.csv', final_points_complete_step, delimiter=',')
        np.savetxt('ros2/src/march_gait_planning/m9_gait_files/cartesian/small_step_close.csv', final_points_step_close, delimiter=',')

def stair_gaits(set_of_points, array_size, pause_time_fraction = 6):
    pause_time = int(array_size/pause_time_fraction)
    first_step_points = np.array(set_of_points[0]["first_step"])
    full_step_points = np.array(set_of_points[1]["full_step"])
    step_close_points = np.array(set_of_points[2]["step_close"])
    number_of_time_points = np.linspace(0, 1.0, int(array_size/2))

    # Part 1: swing leg goes up one step, stance leg stays on the ground
    first_step_curve = bezier.Curve(first_step_points.T, degree=3)
    points= first_step_curve.evaluate_multi(number_of_time_points)

    x_swing_stairs = points[0,:]
    z_swing_stairs = points[1,:]

    x_swing_stairs, z_swing_stairs = make_evenly_spaced_points(x_swing_stairs, z_swing_stairs, int(array_size/2))
    
    x_stance_stairs = np.linspace(first_step_points[0][0], -first_step_points[3][0], int(array_size/2))
    z_stance_stairs = [0]*int(array_size/2)

    x_swing_stairs = np.append(x_swing_stairs, x_swing_stairs[-1]*np.ones(pause_time))
    z_swing_stairs = np.append(z_swing_stairs, z_swing_stairs[-1]*np.ones(pause_time))
    x_stance_stairs = np.append(x_stance_stairs, x_stance_stairs[-1]*np.ones(pause_time))
    z_stance_stairs = np.append(z_stance_stairs, z_stance_stairs[-1]*np.ones(pause_time))

    first_points_stairs = np.column_stack((x_swing_stairs, z_swing_stairs, x_stance_stairs, z_stance_stairs))

    # Part 2: swing leg goes down two steps, stance leg stays still
    full_step_curve = bezier.Curve(full_step_points.T, degree=3)
    points= full_step_curve.evaluate_multi(number_of_time_points)

    x_swing_stair = points[0,:]
    z_swing_stair = points[1,:]

    x_swing_stair, z_swing_stair = make_evenly_spaced_points(x_swing_stair, z_swing_stair, int(array_size/2))
    
    x_stance_stair = np.linspace(full_step_points[3][0], full_step_points[0][0], int(array_size/2))
    z_stance_stair = np.linspace(full_step_points[3][1], full_step_points[0][1], int(array_size/2))

    x_swing_stair = np.append(x_swing_stair, x_swing_stair[-1]*np.ones(pause_time))
    z_swing_stair = np.append(z_swing_stair, z_swing_stair[-1]*np.ones(pause_time))
    x_stance_stair = np.append(x_stance_stair, x_stance_stair[-1]*np.ones(pause_time))
    z_stance_stair = np.append(z_stance_stair, z_stance_stair[-1]*np.ones(pause_time))

    # Part 3: swing leg closes to stance leg (top of the stairs)
    step_close_curve = bezier.Curve(step_close_points.T, degree=3)
    points = step_close_curve.evaluate_multi(number_of_time_points)

    x_swing_last_step = points[0,:]
    z_swing_last_step = points[1,:]

    x_swing_last_step, z_swing_last_step = make_evenly_spaced_points(x_swing_last_step, z_swing_last_step, int(array_size/2))
    
    x_stance_last_step = np.linspace(-step_close_points[0][0], step_close_points[3][0] , int(array_size/2))
    z_stance_last_step = np.linspace(full_step_points[3][1], 0, int(array_size/2))

    second_step_points_stairs = np.column_stack((x_stance_stair, z_stance_stair, x_swing_stair, z_swing_stair))

    third_step_points_stairs = np.column_stack((x_swing_stair, z_swing_stair, x_stance_stair, z_stance_stair))

    forth_step_points_stairs = np.column_stack((x_stance_stair, z_stance_stair, x_swing_stair, z_swing_stair))

    final_step_points_stairs = np.column_stack((x_swing_last_step, z_swing_last_step, x_stance_last_step, z_stance_last_step))



    final_points_stairs_up = np.concatenate((first_points_stairs, second_step_points_stairs, third_step_points_stairs, forth_step_points_stairs, final_step_points_stairs), axis=0)


    # plt.plot(final_points_stairs_up[:,0], final_points_stairs_up[:,1])
    # plt.plot(final_points_stairs_up[:,2], final_points_stairs_up[:,3], color="orange")
    # plt.show()

    return final_points_stairs_up

def descend_stairs(set_of_points, array_size, pause_time_fraction = 6): 
    pause_time = int(array_size/pause_time_fraction)
    first_step_points = np.array(set_of_points[0]["first_step"])
    full_step_points = np.array(set_of_points[1]["full_step"])
    step_close_points = np.array(set_of_points[2]["step_close"])
    number_of_time_points = np.linspace(0, 1.0, int(array_size/2))

    # Part 1: swing leg goes down one step, stance leg stays on the ground
    first_step_curve = bezier.Curve(first_step_points.T, degree=3)
    points= first_step_curve.evaluate_multi(number_of_time_points)

    x_swing_first_step = points[0,:]
    z_swing_first_step = points[1,:]

    x_swing_first_step, z_swing_first_step = make_evenly_spaced_points(x_swing_first_step, z_swing_first_step, int(array_size/2))
    
    x_stance_first_step = np.linspace(first_step_points[0][0], -first_step_points[3][0], int(array_size/2))
    z_stance_first_step = np.concatenate((np.zeros(int(array_size/4)), np.linspace(0, full_step_points[0][1], int(array_size/4))))

    x_swing_first_step = np.append(x_swing_first_step, x_swing_first_step[-1]*np.ones(pause_time))
    z_swing_first_step = np.append(z_swing_first_step, z_swing_first_step[-1]*np.ones(pause_time))
    x_stance_first_step = np.append(x_stance_first_step, x_stance_first_step[-1]*np.ones(pause_time))
    z_stance_first_step = np.append(z_stance_first_step, z_stance_first_step[-1]*np.ones(pause_time))

    # Part 2: swing leg goes down two steps, stance leg stays still
    full_step_curve = bezier.Curve(full_step_points.T, degree=3)
    points= full_step_curve.evaluate_multi(number_of_time_points)

    x_swing_full_step = points[0,:]
    z_swing_full_step = points[1,:]

    x_swing_full_step, z_swing_full_step = make_evenly_spaced_points(x_swing_full_step, z_swing_full_step, int(array_size/2))

    x_stance_full_step = np.linspace(full_step_points[3][0], full_step_points[0][0], int(array_size/2))
    z_stance_full_step = np.concatenate((full_step_points[3][1]*np.ones(int(array_size/4)), np.linspace(full_step_points[3][1], full_step_points[0][1], int(array_size/4))))

    x_swing_full_step = np.append(x_swing_full_step, x_swing_full_step[-1]*np.ones(pause_time))
    z_swing_full_step = np.append(z_swing_full_step, z_swing_full_step[-1]*np.ones(pause_time))
    x_stance_full_step = np.append(x_stance_full_step, x_stance_full_step[-1]*np.ones(pause_time))
    z_stance_full_step = np.append(z_stance_full_step, z_stance_full_step[-1]*np.ones(pause_time))

    # Part 3: swing leg closes to stance leg (top of the stairs)
    step_close_curve = bezier.Curve(step_close_points.T, degree=3)
    points = step_close_curve.evaluate_multi(number_of_time_points)

    x_swing_last_step = points[0,:]
    z_swing_last_step = points[1,:]

    x_swing_last_step, z_swing_last_step = make_evenly_spaced_points(x_swing_last_step, z_swing_last_step, int(array_size/2))
    
    x_stance_last_step = np.linspace(full_step_points[3][0], 0 , int(array_size/2))
    z_stance_last_step = np.linspace(full_step_points[3][1], 0, int(array_size/2))

    first_step_trajectory = np.column_stack((x_swing_first_step, z_swing_first_step, x_stance_first_step, z_stance_first_step))
    second_step_trajectory = np.column_stack((x_stance_full_step, z_stance_full_step, x_swing_full_step, z_swing_full_step))
    third_step_trajectory = np.column_stack((x_swing_full_step, z_swing_full_step, x_stance_full_step, z_stance_full_step))
    forth_step_trajectory = np.column_stack((x_stance_full_step, z_stance_full_step, x_swing_full_step, z_swing_full_step))
    final_step_trajectory = np.column_stack((x_swing_last_step, z_swing_last_step, x_stance_last_step, z_stance_last_step))

    final_points_stairs_down = np.concatenate((first_step_trajectory, second_step_trajectory, third_step_trajectory, forth_step_trajectory, final_step_trajectory), axis=0)
    
    # plt.plot(final_points_stairs_down[:,0], final_points_stairs_down[:,1])
    # plt.plot(final_points_stairs_down[:,2], final_points_stairs_down[:,3], color="orange")
    # plt.show()

    return final_points_stairs_down

def high_step_up(bezier_points, array_size):
    # array_size is the time for one swing phase

    pause_time = int(array_size/4) # make pause longer 
    step_length = bezier_points.max(axis=0)[0]
    ending_angle = np.arcsin((step_length)/LEG_LENGTH)
    phi_first_step = np.linspace(0, ending_angle, int(array_size))

    # ---------------------------------- Part 1, first step -------------------------------------------------------------
    # The stance leg follows a circle with radius LEG_LENGTH with center at (0, LEG_LENGTH). Sine and cosine are swapped from a usual circle, since we want the circle to start at the bottom and go backwards
    x_stance_first_step = -LEG_LENGTH*np.sin(phi_first_step)
    z_stance_first_step = LEG_LENGTH - LEG_LENGTH*np.cos(phi_first_step)

    x_stance_first_step = np.append(x_stance_first_step, x_stance_first_step[-1]*np.ones(pause_time))
    z_stance_first_step = np.append(z_stance_first_step, z_stance_first_step[-1]*np.ones(pause_time))

    vertical_offset = z_stance_first_step[-1]

    first_step_bezier_points = bezier_points.copy()
    first_step_bezier_points[1:, 1] = first_step_bezier_points[1:, 1] + vertical_offset

    curve_first_step = bezier.Curve(first_step_bezier_points.T, degree=3)
    number_of_time_points_first_step = np.linspace(0, 1.0, int(array_size))
    points_first_step= curve_first_step.evaluate_multi(number_of_time_points_first_step)

    x_swing_first_step = points_first_step[0,:]
    z_swing_first_step = points_first_step[1,:]

    x_swing_first_step, z_swing_first_step = make_evenly_spaced_points(x_swing_first_step, z_swing_first_step, int(array_size))

    x_swing_first_step = np.append(x_swing_first_step, x_swing_first_step[-1]*np.ones(pause_time))
    z_swing_first_step = np.append(z_swing_first_step, z_swing_first_step[-1]*np.ones(pause_time))

    # --------------------------------- Part 2, step close ----------------------------------------------------------------
    x_stance_step_close = np.linspace(x_swing_first_step[-1], 0, int(array_size))
    z_stance_step_close = np.linspace(z_swing_first_step[-1], 0, int(array_size))

    step_close_bezier_points = bezier_points.copy()
    step_close_bezier_points[3, 1] = vertical_offset

    curve_step_close  = bezier.Curve(step_close_bezier_points.T, degree=3)
    number_of_time_points_step_close = np.linspace(0, 1.0, int(array_size))
    points_step_close = curve_step_close.evaluate_multi(number_of_time_points_step_close)

    x_swing_step_close = points_step_close[0,:] + x_stance_first_step[-1]
    z_swing_step_close = np.flip(points_step_close[1,:])


    # Combine everything
    x_right_high_step = np.append(x_swing_first_step, x_stance_step_close)
    z_right_high_step = np.append(z_swing_first_step, z_stance_step_close)
    x_left_high_step = np.append(x_stance_first_step, x_swing_step_close)
    z_left_high_step = np.append(z_stance_first_step, z_swing_step_close)

    print(x_right_high_step.shape, z_right_high_step.shape, x_left_high_step.shape, z_left_high_step.shape)
    

    final_points_high_step_up = np.column_stack((x_right_high_step, z_right_high_step, x_left_high_step, z_left_high_step))

    # plt.plot(x_right_high_step, z_right_high_step)
    # plt.plot(x_left_high_step, z_left_high_step, color="orange")
    # plt.show()

    return final_points_high_step_up

def high_step_down(bezier_points, array_size):
    dataset = high_step_up(bezier_points, array_size)
    x_stance = dataset[:,0][::-1]*-1
    z_stance = dataset[:,1][::-1]
    x_swing = dataset[:,2][::-1]*-1
    z_swing = dataset[:,3][::-1]

    down_step = np.column_stack((x_swing, z_swing, x_stance, z_stance))

    plt.plot(x_swing, z_swing)
    plt.plot(x_stance, z_stance, color="orange")
    plt.show()

    return down_step


def create_high_step_csv(bezier_points: np.array, gait_type: str, array_size: int):
    step_up = high_step_up(bezier_points, array_size)
    step_down = high_step_down(step_up) # make step down quicker 

    if gait_type == "high_step_1":
        np.savetxt('ros2/src/march_gait_planning/m9_gait_files/cartesian/high_step1.csv', step_up, delimiter=',')
        np.savetxt('ros2/src/march_gait_planning/m9_gait_files/cartesian/high_step1_down.csv', step_down, delimiter=',')
    elif gait_type == "high_step_2":
        np.savetxt('ros2/src/march_gait_planning/m9_gait_files/cartesian/high_step2.csv', step_up, delimiter=',')
        np.savetxt('ros2/src/march_gait_planning/m9_gait_files/cartesian/high_step2_down.csv', step_down, delimiter=',')
    elif gait_type == "high_step_3":
        np.savetxt('ros2/src/march_gait_planning/m9_gait_files/cartesian/high_step3.csv', step_up, delimiter=',')
        np.savetxt('ros2/src/march_gait_planning/m9_gait_files/cartesian/high_step3_down.csv', step_down, delimiter=',')
