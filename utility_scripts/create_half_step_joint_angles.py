import pandas as pd 
import matplotlib.pyplot as plt 
import numpy as np

def extract_half_gait():
    full_step = pd.read_csv('./ros2/src/march_gait_planning/m9_gait_files/joint_angles/cartesian_in_joint_angle_full_step.csv')
    step_close = pd.read_csv('./ros2/src/march_gait_planning/m9_gait_files/joint_angles/cartesian_in_joint_angle_step_close.csv')
    first_step = pd.read_csv('./ros2/src/march_gait_planning/m9_gait_files/joint_angles/cartesian_in_joint_angle_first_step.csv')
    full_step.columns=['LHAA', 'LHFE', 'LKFE', 'LADPF', 'RHAA', 'RHFE', 'RKFE', 'RADPF']
    step_close.columns=['LHAA', 'LHFE', 'LKFE', 'LADPF', 'RHAA', 'RHFE', 'RKFE', 'RADPF']
    first_step.columns=['LHAA', 'LHFE', 'LKFE', 'LADPF', 'RHAA', 'RHFE', 'RKFE', 'RADPF']
    half_step = full_step[0:200]
    step_close_updated = step_close[197::]
    first_step_updated = first_step[0:99]
    plot_joints(first_step)
    plot_joints(first_step_updated)
    plot_joints(half_step)
    # plot_joints(full_step)
    plot_joints(step_close_updated)
    return half_step, step_close_updated, first_step_updated


def plot_joints(dataset):

    plt.plot(dataset['LHAA'], label='LHAA', color = 'red')
    plt.plot(dataset['LHFE'], label='LHFE', color = 'fuchsia')
    plt.plot(dataset['LKFE'], label='LKFE', color = 'green')
    plt.plot(dataset['LADPF'], label='LADPF', color = 'blue')

    plt.plot(dataset['RHAA'], label='RHAA', color = 'orange')
    plt.plot(dataset['RHFE'], label='RHFE', color = 'plum')
    plt.plot(dataset['RKFE'], label='RKFE', color = 'lime')
    plt.plot(dataset['RADPF'], label='RADPF', color = 'navy')

    plt.legend()
    plt.show()

half_step, step_close_updated, first_step_updated = extract_half_gait()
# half_step.to_csv('./ros2/src/march_gait_planning/m9_gait_files/joint_angles/cartesian_in_joint_angle_half_step.csv', sep=',', header=False, index=False)
# step_close_updated.to_csv('./ros2/src/march_gait_planning/m9_gait_files/joint_angles/cartesian_in_joint_angle_step_close_updated.csv', sep=',', header=False, index=False)
first_step_updated.to_csv('./ros2/src/march_gait_planning/m9_gait_files/joint_angles/cartesian_in_joint_angle_first_step_updated.csv', sep=',', header=False, index=False)