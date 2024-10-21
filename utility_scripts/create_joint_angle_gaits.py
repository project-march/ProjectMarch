import pandas as pd 
import matplotlib.pyplot as plt 
import numpy as np
from scipy.interpolate import CubicSpline, UnivariateSpline, interp1d
from scipy.signal import argrelmax, argrelmin
import yaml 
import os

# Constants
BASE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'ros2', 'src', 'march_gait_planning', 'm9_gait_files', 'joint_angles'))
COLUMNS = ['LADPF', 'LHAA', 'LHFE', 'LKFE', 'RADPF', 'RHAA', 'RHFE', 'RKFE']
HIGH_LEVEL_FREQUENCY = 200
HOMESTAND_FILE = 'homestand.yaml'


def ensure_directory_exists(path):
    """Ensure the directory for the given path exists."""
    os.makedirs(os.path.dirname(path), exist_ok=True)

def save_to_csv(data, filename):
    """Save the given data to a CSV file in the BASE_DIR."""
    file_path = os.path.join(BASE_DIR, filename)
    ensure_directory_exists(file_path)
    np.savetxt(file_path, data, delimiter=',')
    print(f"Saved data to {file_path}")

def import_homestand(): 
    homestand_path = os.path.abspath(os.path.join(BASE_DIR, '..', HOMESTAND_FILE))
    if os.path.exists(homestand_path):
        with open(homestand_path, 'r') as f:
            data = yaml.safe_load(f)
            homestand_joint_angles = np.array(data["joint_angles"])
            return homestand_joint_angles
    else: 
        raise FileNotFoundError(f"The file {homestand_path} does not exist")
    
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

def stand_to_sit(time_points, train_sit=False):
    homestand = import_homestand()

    # Set end points based on train_sit if True
    end_points = [0.087, -0.065, 1.75, 1.75] if train_sit else [0.087, -0.065, 1.57, 1.57]

    # Ensure time_points is an integer
    time_points = int(time_points)

    # Generate trajectories
    trajectories = [np.linspace(homestand[i], end_points[i], time_points) for i in range(4)]
    adpf_trajectory, haa_trajectory, hfe_trajectory, kfe_trajectory = trajectories

    # Combine trajectories into a single array
    stand_to_sit_data = np.column_stack([
            adpf_trajectory, haa_trajectory, hfe_trajectory, kfe_trajectory, 
            adpf_trajectory, haa_trajectory, hfe_trajectory, kfe_trajectory
        ])

    # Save to CSV file
    filename = 'stand_to_trainsit.csv' if train_sit else 'stand_to_sit.csv'
    save_to_csv(stand_to_sit_data, filename)

def sit_to_stand(time_points, train_sit=False):
    homestand = import_homestand()
    
    # Set starting points based on train_sit if True
    starting_points = [0.087, -0.065, 1.75, 1.75] if train_sit else [0.087, -0.065, 1.57, 1.57]

    # Ensure time_points is an integer
    time_points = int(time_points)

    # Generate trajectories
    trajectories = [np.linspace(starting_points[i], homestand[i], time_points) for i in range(4)]
    adpf_trajectory, haa_trajectory, hfe_trajectory, kfe_trajectory = trajectories

    # Combine trajectories into a single array
    sit_to_stand_data = np.column_stack([
        adpf_trajectory, haa_trajectory, hfe_trajectory, kfe_trajectory, 
        adpf_trajectory, haa_trajectory, hfe_trajectory, kfe_trajectory
    ])

    # Save to CSV file
    filename = 'trainsit_to_stand.csv' if train_sit else 'sit_to_stand.csv'
    save_to_csv(sit_to_stand_data, filename)  
    
# Hinge gait, also known as cringe gait
def hinge_gait(time_points):
    homestand = import_homestand()
    end_points = [homestand[0], homestand[1], 1.5, homestand[3]]

    # Ensure time_points is an integer
    time_points = int(time_points)

    trajectories = [np.linspace(homestand[i], end_points[i], time_points) for i in range(4)]
    adpf_trajectory, haa_trajectory, hfe_trajectory, kfe_trajectory = trajectories

    hinge_gait_data = np.column_stack([
        adpf_trajectory, haa_trajectory, hfe_trajectory, kfe_trajectory, 
        adpf_trajectory, haa_trajectory, hfe_trajectory, kfe_trajectory
    ])

    save_to_csv(hinge_gait_data, 'hinge_gait.csv')

# Side right
def sideways(time_points):
    homestand = import_homestand()
    pause_time_adjustment = 50
    pause_time = int(time_points / 2 + pause_time_adjustment)
    sideways_step_size = 0.20

    ladpf_h, lhaa_h, lhfe_h, lkfe_h, radpf_h, rhaa_h, rhfe_h, rkfe_h = homestand
    
    # Ensure time_points is an integer
    time_points = int(time_points)

    ladpf1 = np.full(time_points, homestand[0])
    lhaa1 = np.linspace(lhaa_h, -sideways_step_size, time_points)
    lhfe1 = np.full(time_points, lhfe_h)
    lkfe1 = np.linspace(lkfe_h, lkfe_h, time_points)

    adpf_swing_up = np.linspace(radpf_h, 0.20, int(time_points / 2))
    adpf_swing_down = np.linspace(0.20, radpf_h, int(time_points / 2))
    radpf1 = np.append(adpf_swing_up, adpf_swing_down)
    
    rhaa1 = np.linspace(rhaa_h, -sideways_step_size, time_points)

    hfe_swing_up = np.linspace(rhfe_h, 0.45, int(time_points / 2))
    hfe_swing_down = np.linspace(0.45, rhfe_h, int(time_points / 2))
    rhfe1 = np.append(hfe_swing_up, hfe_swing_down)

    kfe_swing_up = np.linspace(rkfe_h, 0.9, int(time_points / 2))
    kfe_swing_down = np.linspace(0.9, rkfe_h, int(time_points / 2)) 
    rkfe1 = np.append(kfe_swing_up, kfe_swing_down)
    
    right_open = np.column_stack([
        ladpf1, lhaa1, lhfe1, lkfe1, radpf1, rhaa1, rhfe1, rkfe1
    ])

    # Add pause to enable swapping of stance leg
    right_open = np.append(right_open, np.tile(right_open[-1, :], (pause_time, 1)), axis=0)

    # Right stance leg
    radpf = np.full(time_points, radpf_h)
    rhaa = np.linspace(-sideways_step_size, rhaa_h, time_points)
    rhfe = np.full(time_points, rhfe_h)
    rkfe = np.full(time_points, rkfe_h)

    ladpf = np.append(adpf_swing_up, adpf_swing_down)
    lhaa = np.linspace(-sideways_step_size, lhaa_h, time_points)
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

    full_side_left = np.hstack((full_side_right[:, 4:8], full_side_right[:, 0:4]))
    are_arrays_equal = np.array_equal(full_side_left[:, 0:3], full_side_right[:, 4:7])
    print(f"Are the left and right side arrays equal? {'Yes' if are_arrays_equal else 'No'}")

    if are_arrays_equal:
        save_to_csv(full_side_left, 'sidestep_left.csv')
        save_to_csv(full_side_right, 'sidestep_right.csv')
    else:
        print("The arrays are not equal. Both sidestepping files will not be saved.")


# Uncomment the gait you want to generate
# hinge_gait(HIGH_LEVEL_FREQUENCY * 0.5)
# stand_to_sit(HIGH_LEVEL_FREQUENCY * 2.5)
# sit_to_stand(HIGH_LEVEL_FREQUENCY * 1.5, False)
# sideways(HIGH_LEVEL_FREQUENCY * 2)
