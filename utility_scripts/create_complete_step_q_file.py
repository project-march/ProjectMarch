import pandas as pd 
import matplotlib.pyplot as plt 
import numpy as np

df_gait_joint = pd.read_csv('./ros2/src/march_gait_planning/m9_gait_files/joint_angles/q_test.csv')


df_gait_joint = df_gait_joint.assign(LHAA=-0.06)
df_gait_joint = df_gait_joint.assign(RHAA=-0.06)
df_gait_joint = df_gait_joint.assign(LADPF=0.119176)
df_gait_joint = df_gait_joint.assign(RADPF=0.077083)


df_gait_joint['LKFE'] = df_gait_joint['LKFE']*-1
df_gait_joint['RKFE'] = df_gait_joint['RKFE']*-1
df_gait_joint['LHFE'] = df_gait_joint['LHFE']
df_gait_joint['RHFE'] = df_gait_joint['RHFE']

df_gait_joint.drop(df_gait_joint.index[0:50], inplace=True)
df_gait_joint.drop(df_gait_joint.index[350:567], inplace=True)

df_gait_joint = df_gait_joint.reset_index(drop=True)

print(f"Index of max RHFE: ", df_gait_joint[['RHFE']].idxmax())
print(f"Index of 2nd max RHFE: ", df_gait_joint[['RHFE']].iloc[150:250].idxmax())

first_step = df_gait_joint.iloc[0:72]
first_step.to_csv('./ros2/src/march_gait_planning/m9_gait_files/joint_angles/first_step_q.csv', sep=',', header=False, index=False)
full_step = df_gait_joint.iloc[73:189]
full_step.to_csv('./ros2/src/march_gait_planning/m9_gait_files/joint_angles/one_step_q.csv', sep=',', header=False, index=False)

plt.plot(first_step['LKFE'], label='LKFE')
plt.plot(first_step['RKFE'], label='RKFE')
plt.plot(first_step['LHFE'], label='LHFE')
plt.plot(first_step['RHFE'], label='RHFE')
plt.legend()
plt.show()

plt.plot(full_step['LKFE'], label='LKFE')
plt.plot(full_step['RKFE'], label='RKFE')
plt.plot(full_step['LHFE'], label='LHFE')
plt.plot(full_step['RHFE'], label='RHFE')
plt.legend()
plt.show()

df_gait_joint.to_csv('./ros2/src/march_gait_planning/m9_gait_files/joint_angles/full_gait_q.csv', sep=',', header=False, index=False)

def stand_to_sit():
    time_points = 100
    lhaa = np.linspace(-0.06, -0.3, time_points)
    lhfe = np.linspace(0.351166, 1.5, time_points)
    lkfe = np.linspace(0.372236, 1.2, time_points)
    ladpf = np.linspace(0.119176, -0.5, time_points)

    rhaa = np.linspace(-0.06, 0.3, time_points)
    rhfe = np.linspace(0.274982, 1.5, time_points)
    rkfe = np.linspace(0.286291, 1.2, time_points)
    radpf = np.linspace(0.077083, -0.5, time_points)

    joint_angles_dataset = np.column_stack([
        lhaa, lhfe, lkfe, ladpf, rhaa, rhfe, rkfe, radpf
    ])

    np.savetxt('./ros2/src/march_gait_planning/m9_gait_files/joint_angles/stand_to_sit.csv', 
               joint_angles_dataset, delimiter=',')

def sit_to_stand():
    time_points = 100
    lhaa = np.linspace(-0.3, -0.06, time_points)
    lhfe = np.linspace(1.5, 0.351166, time_points)
    lkfe = np.linspace(1.2, 0.372236, time_points)
    ladpf = np.linspace(-0.5, 0.119176, time_points)

    rhaa = np.linspace(0.3, -0.06, time_points)
    rhfe = np.linspace(1.5, 0.274982, time_points)
    rkfe = np.linspace(1.2, 0.286291, time_points)
    radpf = np.linspace(-0.5, 0.077083, time_points)

    sit_to_stand = np.column_stack([
        lhaa, lhfe, lkfe, ladpf, rhaa, rhfe, rkfe, radpf
    ])

    np.savetxt('./ros2/src/march_gait_planning/m9_gait_files/joint_angles/sit_to_stand.csv', 
               sit_to_stand, delimiter=',')

    
stand_to_sit()
sit_to_stand()
