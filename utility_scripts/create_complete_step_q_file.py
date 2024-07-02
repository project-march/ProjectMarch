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


    # hinge_gait = np.column_stack([
    #     lhaa, lhfe, lkfe, ladpf, rhaa, rhfe, rkfe, radpf
    # ])

    hinge_gait = np.column_stack([
        ladpf, lhaa, lhfe, lkfe, radpf, rhaa, rhfe, rkfe
    ])

    np.savetxt('./ros2/src/march_gait_planning/m9_gait_files/joint_angles/hinge_gait.csv', 
               hinge_gait, delimiter=',')

def sideways(time_points):
    homestand = import_homestand()

    ladpf_h, lhaa_h, lhfe_h, lkfe_h, radpf_h, rhaa_h, rhfe_h, rkfe_h = homestand


    ladpf1 = np.linspace(ladpf_h, ladpf_h, time_points)
    lhaa1 = np.linspace(lhaa_h, -0.2, time_points)
    lhfe1 = np.linspace(lhfe_h, lhfe_h, time_points)
    lkfe1 = np.linspace(lkfe_h, 0.3, time_points)
    radpf1 = np.linspace(radpf_h, radpf_h, time_points)
    rhaa1 = np.append(np.linspace(rhaa_h, -0.15, int(((2000000000-1500000000)/2000000000)*time_points)), np.linspace(-0.15, -0.2, int(((1500000000)/2000000000)*time_points)))
    rhfe1 = np.linspace(rhfe_h, rhfe_h, time_points)
    rkfe1 = np.linspace(rkfe_h, 0.3, time_points)
    
    left_open = np.column_stack([
        ladpf1, lhaa1, lhfe1, lkfe1, radpf1, rhaa1, rhfe1, rkfe1
    ])

    ladpf = np.linspace(ladpf_h, ladpf_h, time_points)
    lhaa = np.append(np.linspace(-0.2, -0.15, int((1500000000/2000000000)*time_points)), np.linspace(-0.15, lhaa_h, int(((2000000000-1500000000)/2000000000)*time_points)))
    lhfe = np.linspace(lhfe_h, lhfe_h, time_points)
    lkfe = np.linspace(0.3, lkfe_h, time_points)
    radpf = np.linspace(radpf_h, radpf_h, time_points)
    rhaa = np.linspace(-0.2, rhaa_h, time_points)
    rhfe = np.linspace(rhfe_h, rhfe_h, time_points)
    rkfe = np.linspace(0.3, rkfe_h, time_points)
    
    right_close = np.column_stack([
        ladpf, lhaa, lhfe, lkfe, radpf, rhaa, rhfe, rkfe
    ])

    full_sidestep = np.vstack((left_open, right_close)) 

    plt.plot(full_sidestep[:,2], label='LKFE')
    plt.plot(full_sidestep[:,6], label='RKFE')
    plt.plot(full_sidestep[:,1], label='LHFE')
    plt.plot(full_sidestep[:,5], label='RHFE')
    plt.plot(full_sidestep[:,0], label='LHAA')
    plt.plot(full_sidestep[:,4], label='RHAA')
    plt.legend()
    plt.show()
       
    np.savetxt('./ros2/src/march_gait_planning/m9_gait_files/joint_angles/sidestep.csv', 
               full_sidestep, delimiter=',')

def remap_iks_translated_joint_angle_gaits(old_name, new_name):
    df = pd.read_csv(f'./ros2/src/march_gait_planning/m9_gait_files/joint_angles/{old_name}.csv', names = ['LHAA', 'LHFE', 'LKFE', 'LADPF', 'RHAA', 'RHFE', 'RKFE', 'RADPF'])
    # print(df)
    df = df[COLUMNS]
    # print(df)
    df.to_csv(f'./ros2/src/march_gait_planning/m9_gait_files/joint_angles/{new_name}.csv', sep=',', header=False, index=False)

    
# hinge_gait(100)
# stand_to_sit(125)
# sit_to_stand(125)
sideways(400)



# remap_iks_translated_joint_angle_gaits('cartesian_in_joint_angle_first_step_updated', 'cartesian_in_joint_angle_first_step')
# remap_iks_translated_joint_angle_gaits('cartesian_in_joint_angle_step_close_updated', 'cartesian_in_joint_angle_step_close')
# remap_iks_translated_joint_angle_gaits('cartesian_in_joint_angle_half_step', 'cartesian_in_joint_angle_half_step_def')
