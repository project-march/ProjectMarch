import numpy as np
import matplotlib.pyplot as plt
import time

import quadrilateral_angle_solver as qas
import triangle_angle_solver as tas
from goniometric_functions_degrees import cos, sin, asin

upper_leg = 41
lower_leg = 41
foot_length = 10
total_leg = upper_leg + lower_leg


def calculate_joint_positions(pose):
    """
    Calculates the joint positions for a given pose, were:
    - ankle1 is at [0,0]
    - toe1 is at [foot_length, 0], since the foot is flat.
    - knee1 is at ankle1 + lower_leg in direction given by ankle1
    - hip is at knee1 + upper_leg in direction given by ankle and knee1.
    - knee2 is at hip + upper_leg in direction given by hip_fe2
    - ankle2 is at knee2 + lower_leg in direction given by hip_fe2 and knee2
    - toe2 is at ankle2 + foot_length in direction given by hip_fe2, knee2 and ankle2
    """

    toe1 = np.array([foot_length, 0])
    ankle1 = np.array([0, 0])
    knee1 = np.array(
        [ankle1[0] + sin(pose[0]) * lower_leg, ankle1[1] + cos(pose[0]) * lower_leg]
    )
    hip = np.array(
        [
            knee1[0] + sin(pose[0] - pose[1]) * upper_leg,
            knee1[1] + cos(pose[0] - pose[1]) * upper_leg,
        ]
    )
    knee2 = np.array(
        [hip[0] + sin(pose[3]) * upper_leg, hip[1] - cos(pose[3]) * upper_leg]
    )
    ankle2 = np.array(
        [
            knee2[0] + sin(pose[3] - pose[4]) * lower_leg,
            knee2[1] - cos(pose[3] - pose[4]) * lower_leg,
        ]
    )
    hip_and_knee_angle = pose[3] - pose[4]
    toe2 = np.array(
        [
            ankle2[0] + sin(hip_and_knee_angle + 90 + pose[5]) * foot_length,
            ankle2[1] - cos(hip_and_knee_angle + 90 + pose[5]) * foot_length,
        ]
    )

    return toe1, ankle1, knee1, hip, knee2, ankle2, toe2


def make_plot(pose):
    """
    Makes a plot of the exo by first calculating the joint positions
    and then plotting them with lines.
    """

    toe1, ankle1, knee1, hip, knee2, ankle2, toe2 = calculate_joint_positions(pose)

    plt.figure(1)
    plt.plot(
        [toe1[0], ankle1[0], knee1[0], hip[0], knee2[0], ankle2[0], toe2[0]],
        [toe1[1], ankle1[1], knee1[1], hip[1], knee2[1], ankle2[1], toe2[1]],
    )
    plt.gca().set_aspect("equal", adjustable="box")
    plt.show()


def calculate_ground_pose_flexion(ankle_x):
    """
    Calculates and returns the flexion of the ankles and the hips when the
    ankle is moved to a certain x position, using pythagoras theorem.
    """

    return asin((ankle_x / 2) / total_leg)


def calculate_lifted_hip_ankle_distance(ground_angle, ankle):
    """
    Calculates th distance between the hip and the lifted ankle.
    """

    hip = np.array([cos(ground_angle) * total_leg, sin(ground_angle) * total_leg])
    return np.linalg.norm(hip - ankle)


def calculate_lifted_pose(ankle, pose):
    """
    Calculate the pose after lifting the foot to the desired ankle postion by:
    1. Make a triangle between hip, knee2 and ankle and calculate angles using side lengths.
    2. hip_fe2 = angle between [vertical, hip and ankle] + hip_angle from step 1
    3. knee2 = 180 - knee_angle from step 1
    4. ankle2 = 90 - (angle between [toe, ankle, hip] - ankle_angle from step 1)
    """

    toe1, ankle1, knee1, hip, knee2, ankle2, toe2 = calculate_joint_positions(pose)
    hip_angle, knee_angle, ankle_angle = tas.get_angles_from_sides(
        [lower_leg, np.linalg.norm(hip - ankle), upper_leg]
    )

    pose[3] = (
        qas.get_angle_between_points([np.array([ankle[0] / 2, 0]), hip, ankle])
        + hip_angle
    )
    pose[4] = 180 - knee_angle
    pose[5] = 90 - (
        qas.get_angle_between_points([ankle + np.array([foot_length, 0]), ankle, hip])
        - ankle_angle
    )

    return pose


def reduce_dorsi_flexion(pose, max_flexion):
    """
    Calculate the pose after reducing the dorsiflexion using quadrilateral solver with points:
    a = knee1, b = ankle2, c = knee2, d = hip, given the desired angle of b.
    """

    if pose[5] > max_flexion:

        # Get current state:
        toe1, ankle1, knee1, hip, knee2, ankle2, toe2 = calculate_joint_positions(pose)

        # Determine first angle of quadrilateral:
        reduction = pose[5] - max_flexion
        b_angle = qas.get_angle_between_points([knee1, ankle2, knee2]) - reduction

        da = upper_leg
        ab = np.linalg.norm(knee1 - ankle2)
        bc = lower_leg
        cd = upper_leg
        lengths = [da, ab, bc, cd]
        knee1_angle, ankle2_angle, knee2_angle, hip_angle = qas.solve_quadritlateral(
            lengths, b_angle
        )

        # Define new pose:
        pose[1] = (
            knee1_angle + qas.get_angle_between_points([ankle2, knee1, ankle1]) - 180
        )

        # Get new hip location:
        toe1, ankle1, knee1, hip, knee2, ankle2, toe2 = calculate_joint_positions(pose)

        vertical = np.array([hip[0], knee1[1]])

        if hip[0] < knee1[0]:
            pose[2] = qas.get_angle_between_points([knee1, hip, vertical])
        else:
            pose[2] = -qas.get_angle_between_points([knee1, hip, vertical])

        pose[3] = hip_angle + pose[2]
        pose[4] = 180 - knee2_angle
        pose[5] -= reduction

    return pose


def straighten_leg(pose):
    """
    Straighten stance leg by making a triangle between ankle1, hip and knee2
    and calculating new angles.
    """

    # Get current state:
    toe1, ankle1, knee1, hip, knee2, ankle2, toe2 = calculate_joint_positions(pose)

    # Determine sides of triangle:
    a = upper_leg  # Distance from knee2 to hip
    b = total_leg  # Distance from ankle1 to hip with straight leg
    c = np.linalg.norm(ankle1 - knee2)  # Distance form ankle1 to knee2
    sides = [a, b, c]

    # Get triangle angles from sides:
    ankle_angle, knee_angle, hip_angle = tas.get_angles_from_sides(sides)

    # Define new pose:
    pose[0] = 90 - (ankle_angle + qas.get_angle_between_points([knee2, ankle1, toe1]))
    pose[1] = 0

    # Get new knee1 and hip location:
    toe1, ankle1, knee1, hip, knee2, ankle2, toe2 = calculate_joint_positions(pose)

    vertical = np.array([hip[0], knee1[1]])

    if hip[0] < knee1[0]:
        pose[2] = qas.get_angle_between_points([knee1, hip, vertical])
    else:
        pose[2] = -qas.get_angle_between_points([knee1, hip, vertical])

    pose[3] = hip_angle + pose[2]
    pose[4] = 180 - (knee_angle + qas.get_angle_between_points([ankle1, knee2, ankle2]))

    return pose


def solve_ik(
    ankle_position, max_flexion, make_end_plot=False, make_all_plots=False, timer=False
):
    """
    Solve inverse kinematics for a desired ankle location, assuming flat feet.
    Expects the ankle position and max dorsi flexion and returns the calculated pose.
    """
    start = time.time()

    # Calculate ground pose:
    ground_pose_flexion = calculate_ground_pose_flexion(ankle_position[0])
    pose = [
        ground_pose_flexion,
        0,
        -ground_pose_flexion,
        ground_pose_flexion,
        0,
        -ground_pose_flexion,
    ]

    if make_all_plots:
        make_plot(pose)

    # Calculate lifted pose:
    pose = calculate_lifted_pose(ankle_position, pose)

    if make_all_plots:
        make_plot(pose)

    # Reduce dorsi flexion:
    pose = reduce_dorsi_flexion(pose, max_flexion)

    if make_all_plots:
        make_plot(pose)

    # Straighten leg:
    pose = straighten_leg(pose)

    if timer:
        end = time.time()
        print("Calculation time = ", end - start, " seconds")

    if make_all_plots or make_end_plot:
        make_plot(pose)

    return pose
