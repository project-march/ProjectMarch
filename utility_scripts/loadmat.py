# import scipy.io as sio

# test = sio.loadmat('/home/control/Downloads/F_AMASS/AMASS/F_amass_Subject_1.mat')

# teststruct = test['Subject_1_F_amass']
# print("types in file: ", teststruct.dtype)
# teststruct2move = teststruct[0,0]['move']
# teststruct2subject = teststruct[0,0]['subject']
# teststruct2id = teststruct[0,0]['id']

# print(teststruct2move[12,0])

# sidegaitmove = teststruct2move[12,0][0,0]
# sidegaitsubject = teststruct2subject[0,0]


# for i in range(len(sidegaitmove)):
#     print(f"Content of {i}th item: ", sidegaitmove[i])
#     print(f"Shape of {i}th item: ", sidegaitmove[i].shape)

import numpy as np
import pandas as pd

# Set the number of time points
num_time_points = 400

# Initialize arrays to store joint angles
left_hip_add_abd = np.zeros(num_time_points)
left_hip_flex_ext = np.zeros(num_time_points)
left_knee_flex_ext = np.zeros(num_time_points)
left_ankle_dorsi_plantar = np.zeros(num_time_points)

right_hip_add_abd = np.zeros(num_time_points)
right_hip_flex_ext = np.zeros(num_time_points)
right_knee_flex_ext = np.zeros(num_time_points)
right_ankle_dorsi_plantar = np.zeros(num_time_points)

# Set the range of motion for each joint in radians
hip_add_abd_range = np.radians(20)
hip_flex_ext_range = np.radians(30)
knee_flex_ext_range = np.radians(60)
ankle_dorsi_plantar_range = np.radians(20)

# Generate joint angles for each time point
for t in range(num_time_points):
    # Sideways walking pattern
    phase = 2 * np.pi * t / num_time_points
    sideways_walk_offset = np.radians(10) * np.sin(phase)

    # Left leg joint angles
    left_hip_add_abd[t] = sideways_walk_offset + 0.5 * hip_add_abd_range * np.sin(2 * np.pi * t / num_time_points)
    left_hip_flex_ext[t] = np.radians(20) * np.cos(2 * np.pi * t / num_time_points)
    left_knee_flex_ext[t] = 0.5 * knee_flex_ext_range * np.sin(4 * np.pi * t / num_time_points)
    left_ankle_dorsi_plantar[t] = 0.5 * ankle_dorsi_plantar_range * np.sin(2 * np.pi * t / num_time_points)

    # Right leg joint angles
    right_hip_add_abd[t] = -sideways_walk_offset + 0.5 * hip_add_abd_range * np.sin(2 * np.pi * t / num_time_points)
    right_hip_flex_ext[t] = np.radians(20) * np.cos(2 * np.pi * t / num_time_points)
    right_knee_flex_ext[t] = 0.5 * knee_flex_ext_range * np.sin(4 * np.pi * t / num_time_points)
    right_ankle_dorsi_plantar[t] = 0.5 * ankle_dorsi_plantar_range * np.sin(2 * np.pi * t / num_time_points)

# Create a DataFrame to store the joint angle dataset
data = {
    'LHAA': left_hip_add_abd,
    'LHFE': left_hip_flex_ext,
    'LKFE': left_knee_flex_ext,
    'LADPF': left_ankle_dorsi_plantar,
    'RHAA': right_hip_add_abd,
    'RHFE': right_hip_flex_ext,
    'RKFE': right_knee_flex_ext,
    'RADPF': right_ankle_dorsi_plantar,
}

df = pd.DataFrame(data)

# Save the dataset to a CSV file
df.to_csv('ros2/src/march_gait_planning/m9_gait_files/sideways_walking_gait_dataset.csv', index=False, header=False)