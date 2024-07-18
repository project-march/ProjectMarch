import pandas as pd 
import matplotlib.pyplot as plt 
import numpy as np
from scipy.interpolate import CubicSpline, UnivariateSpline, interp1d
from scipy.signal import argrelmax, argrelmin
import yaml 
import os

COLUMNS = ['LADPF', 'LHAA', 'LHFE', 'LKFE', 'RADPF', 'RHAA', 'RHFE', 'RKFE']

def import_homestand(): 
    current_dir = os.path.dirname(__file__)
    homestand_path = os.path.join(current_dir, '..', 'ros2', 'src', 'march_gait_planning', 'm9_gait_files', 'homestand.yaml')
    if os.path.exists(homestand_path):
        with open(homestand_path, 'r') as f:
            data = yaml.safe_load(f)
            homestand_joint_angles = np.array(data["joint_angles"])
    else: 
        print(f"the file {homestand_path} does not exist")
    
    return homestand_joint_angles
    
def plot_joints(dataset):

    plt.plot(dataset['LKFE'], label='LKFE', color = 'blue')
    plt.plot(dataset['RKFE'], label='RKFE', color = 'green')
    plt.plot(dataset['LHFE'], label='LHFE', color = 'red')
    plt.plot(dataset['RHFE'], label='RHFE', color = 'orange')
    # plt.plot(dataset['LADPF'], label='LADPF')
    # plt.plot(dataset['RADPF'], label='RADPF')
    # plt.plot(dataset['LHAA'], label='LHAA')
    # plt.plot(dataset['RHAA'], label='RHAA')
    plt.legend()
    plt.show()

def stand_to_sit(time_points):

    homestand = import_homestand()
    end_points = [0.087, -0.065, 1.57, 1.57]

    trajectories = [np.linspace(homestand[i], end_points[i], time_points) for i in range(4)]
    adpf_trajectory, haa_trajectory, hfe_trajectory, kfe_trajectory = trajectories

    stand_to_sit = np.column_stack([
            adpf_trajectory, haa_trajectory, hfe_trajectory, kfe_trajectory, adpf_trajectory, haa_trajectory, hfe_trajectory, kfe_trajectory
        ])

    np.savetxt('./ros2/src/march_gait_planning/m9_gait_files/joint_angles/stand_to_sit.csv', 
               stand_to_sit, delimiter=',')

def sit_to_stand(time_points):

    homestand = import_homestand()
    end_points = [0.087, -0.065, 1.57, 1.57]

    trajectories = [np.linspace(end_points[i], homestand[i], time_points) for i in range(4)]
    adpf_trajectory, haa_trajectory, hfe_trajectory, kfe_trajectory = trajectories

    sit_to_stand = np.column_stack([
        adpf_trajectory, haa_trajectory, hfe_trajectory, kfe_trajectory, adpf_trajectory, haa_trajectory, hfe_trajectory, kfe_trajectory
    ])

    np.savetxt('./ros2/src/march_gait_planning/m9_gait_files/joint_angles/sit_to_stand.csv', 
               sit_to_stand, delimiter=',')
    
def hinge_gait(time_points):
    homestand = import_homestand()
    lhaa = np.linspace(-0.03, -0.03, time_points)
    lhfe = np.linspace(0.136, 1.5, time_points)
    lkfe = np.linspace(0.385, 0.385, time_points)
    ladpf = np.linspace(0.162, 0.162, time_points)

    rhaa = np.linspace(-0.013, -0.013, time_points)
    rhfe = np.linspace(0.176, 1.5, time_points)
    rkfe = np.linspace(0.468, 0.468, time_points)
    radpf = np.linspace(0.162, 0.162, time_points)

    hinge_gait = np.column_stack([
        ladpf, lhaa, lhfe, lkfe, radpf, rhaa, rhfe, rkfe
    ])

    np.savetxt('./ros2/src/march_gait_planning/m9_gait_files/joint_angles/hinge_gait.csv', 
               hinge_gait, delimiter=',')

# Side right
def sideways(time_points):
    homestand = import_homestand()
    pause_time = int(time_points/2)

    ladpf_h, lhaa_h, lhfe_h, lkfe_h, radpf_h, rhaa_h, rhfe_h, rkfe_h = homestand

    ladpf1 = np.full(time_points, homestand[0])
    lhaa1 = np.linspace(lhaa_h, -0.25, time_points)
    lhfe1 = np.full(time_points, lhfe_h)
    lkfe1 = np.linspace(lkfe_h, lkfe_h, time_points)

    # Adpf should go up and then back down at the end, kfe should have small flexion and then back to homestand
    adpf_swing_up = np.linspace(radpf_h, 0.20, int(time_points/2))
    adpf_swing_down = np.linspace(0.20, radpf_h, int(time_points/2))
    radpf1 = np.append(adpf_swing_up, adpf_swing_down)
    
    rhaa1 = np.linspace(rhaa_h, -0.25, time_points)

    hfe_swing_up = np.linspace(rhfe_h, 0.45, int(time_points/2))
    hfe_swing_down = np.linspace(0.45, rhfe_h, int(time_points/2))
    rhfe1 = np.append(hfe_swing_up, hfe_swing_down)

    kfe_swing_up = np.linspace(rkfe_h, 0.9, int(time_points/2))
    kfe_swing_down = np.linspace(0.9, rkfe_h, int(time_points/2)) 
    rkfe1 = np.append(kfe_swing_up, kfe_swing_down)
    
    right_open = np.column_stack([
        ladpf1, lhaa1, lhfe1, lkfe1, radpf1, rhaa1, rhfe1, rkfe1
    ])

    # Add pause to enable swapping of stance leg
    right_open = np.append(right_open, np.tile(right_open[-1, :], (pause_time, 1)), axis=0)

    # Right stance leg
    radpf = np.full(time_points, radpf_h)
    rhaa = np.linspace(-0.25, rhaa_h, time_points)
    rhfe = np.full(time_points, rhfe_h)
    rkfe = np.full(time_points, rkfe_h)


    ladpf = np.append(adpf_swing_up, adpf_swing_down)
    lhaa =  np.linspace(-0.25, lhaa_h, time_points)
    lhfe = np.append(hfe_swing_up, hfe_swing_down)
    lkfe = np.append(kfe_swing_up, kfe_swing_down)

    
    left_close = np.column_stack([
        ladpf, lhaa, lhfe, lkfe, radpf, rhaa, rhfe, rkfe
    ])

    # Add pause to enable swapping of stance leg
    left_close = np.append(left_close, np.tile(left_close[-1, :], (pause_time, 1)), axis=0)

    full_side_right = np.vstack((right_open, left_close)) 

    for i, label in enumerate(COLUMNS):
        plt.plot(full_side_right[:, i], label=label)
    plt.legend()
    plt.show()
       
    np.savetxt('./ros2/src/march_gait_planning/m9_gait_files/joint_angles/sidestep_right.csv', 
               full_side_right, delimiter=',')
    full_side_left = np.hstack((full_side_right[:, 4:8], full_side_right[:, 0:4]))
    print(np.array_equal(full_side_left[:, 0:3], full_side_right[:, 4:7]))

    np.savetxt('./ros2/src/march_gait_planning/m9_gait_files/joint_angles/sidestep_left.csv'
                , full_side_left, delimiter=',')

HIGH_LEVEL_FREQUENCY = 200

# hinge_gait(100)
# stand_to_sit(125)
# sit_to_stand(125)
sideways(HIGH_LEVEL_FREQUENCY * 3)
