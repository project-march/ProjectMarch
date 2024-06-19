import pandas as pd 
import matplotlib.pyplot as plt 
import numpy as np
from scipy.interpolate import CubicSpline, UnivariateSpline, interp1d
from scipy.signal import argrelmax, argrelmin

COLUMNS = ['LADPF', 'LHAA', 'LHFE', 'LKFE', 'RADPF', 'RHAA', 'RHFE', 'RKFE']

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

def stand_to_sit():
    time_points = 70
    lhaa = np.linspace(-0.055, -0.055, time_points)
    lhfe = np.linspace(0.03490, 1.57, time_points)
    lkfe = np.linspace(0.185, 1.57, time_points)
    ladpf = np.linspace(0.162, 0.162, time_points)

    rhaa = np.linspace(-0.055, -0.055, time_points)
    rhfe = np.linspace(0.03490, 1.57, time_points)
    rkfe = np.linspace(0.185, 1.57, time_points)
    radpf = np.linspace(0.162, 0.162, time_points)

    # joint_angles_dataset = np.column_stack([
    #     lhaa, lhfe, lkfe, ladpf, rhaa, rhfe, rkfe, radpf
    # ])

    joint_angles_dataset = np.column_stack([
        ladpf, lhaa, lhfe, lkfe, radpf, rhaa, rhfe, rkfe
    ])

    np.savetxt('./ros2/src/march_gait_planning/m9_gait_files/joint_angles/stand_to_sit.csv', 
               joint_angles_dataset, delimiter=',')

def sit_to_stand():
    time_points = 70
    lhaa = np.linspace(-0.055, -0.055, time_points)
    lhfe = np.linspace(1.57, 0.03490, time_points)
    lkfe = np.linspace(1.57, 0.185, time_points)
    ladpf = np.linspace(0.162, 0.162, time_points)

    rhaa = np.linspace(-0.055, -0.055, time_points)
    rhfe = np.linspace(1.57, 0.03490, time_points)
    rkfe = np.linspace(1.57, 0.185, time_points)
    radpf = np.linspace(0.162, 0.162, time_points)

    # sit_to_stand = np.column_stack([
    #     lhaa, lhfe, lkfe, ladpf, rhaa, rhfe, rkfe, radpf
    # ])

    sit_to_stand = np.column_stack([
        ladpf, lhaa, lhfe, lkfe, radpf, rhaa, rhfe, rkfe
    ])

    np.savetxt('./ros2/src/march_gait_planning/m9_gait_files/joint_angles/sit_to_stand.csv', 
               sit_to_stand, delimiter=',')
    
def hinge_gait():
    time_points = 100
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

def sideways(): 
    times = 100
    ladpf1 = np.linspace(0.162, 0.162, times)
    lhaa1 = np.linspace(-0.03, -0.1745, times)
    lhfe1 = np.linspace(0.136, 0.136, times)
    lkfe1 = np.linspace(0.385, 0.5236, times)
    radpf1 = np.linspace(0.162, 0.162, times)
    rhaa1 = np.append(np.linspace(-0.013, -0.1, int(((2000000000-1500000000)/2000000000)*times)), np.linspace(-0.1, -0.1745, int(((1500000000)/2000000000)*times)))
    rhfe1 = np.linspace(0.176, 0.176, times)
    rkfe1 = np.linspace(0.468, 0.5236, times)
    # left_open = np.column_stack([
    #     lhaa1, lhfe1, lkfe1, ladpf1, rhaa1, rhfe1, rkfe1, radpf1
    # ]) 
    left_open = np.column_stack([
        ladpf1, lhaa1, lhfe1, lkfe1, radpf1, rhaa1, rhfe1, rkfe1
    ])

    ladpf = np.linspace(0.162, 0.162, times)
    lhaa = np.append(np.linspace(-0.1745, -0.1, int((1500000000/2000000000)*times)), np.linspace(-0.1, -0.03, int(((2000000000-1500000000)/2000000000)*times)))
    lhfe = np.linspace(0.136, 0.136, times)
    lkfe = np.linspace(0.5236, 0.385, times)
    radpf = np.linspace(0.162, 0.162, times)
    rhaa = np.linspace(-0.1745, -0.013, times)
    rhfe = np.linspace(0.176, 0.176, times)
    rkfe = np.linspace(0.5236, 0.468, times)
    # right_close = np.column_stack([
    #     lhaa, lhfe, lkfe, ladpf, rhaa, rhfe, rkfe, radpf
    # ])
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

    
# hinge_gait()
stand_to_sit()
sit_to_stand()
# sideways()
# remap_iks_translated_joint_angle_gaits('cartesian_in_joint_angle_first_step_updated', 'cartesian_in_joint_angle_first_step')
# remap_iks_translated_joint_angle_gaits('cartesian_in_joint_angle_step_close_updated', 'cartesian_in_joint_angle_step_close')
# remap_iks_translated_joint_angle_gaits('cartesian_in_joint_angle_half_step', 'cartesian_in_joint_angle_half_step_def')