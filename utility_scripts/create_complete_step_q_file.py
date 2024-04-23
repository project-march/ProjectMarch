import pandas as pd 
import matplotlib.pyplot as plt 
import numpy as np
from scipy.interpolate import CubicSpline, UnivariateSpline, interp1d
from scipy.signal import argrelmax, argrelmin

def normal_large_step(): 
    df_gait_joint = pd.read_csv('./ros2/src/march_gait_planning/m9_gait_files/joint_angles/q_test.csv')

    df_gait_joint = df_gait_joint.assign(LHAA=-0.06)
    df_gait_joint = df_gait_joint.assign(RHAA=-0.06)
    df_gait_joint = df_gait_joint.assign(LADPF=0.119176)
    df_gait_joint = df_gait_joint.assign(RADPF=0.077083)

    df_gait_joint['LKFE'] = df_gait_joint['LKFE']*-1
    df_gait_joint['RKFE'] = df_gait_joint['RKFE']*-1

    df_gait_joint.drop(df_gait_joint.index[0:50], inplace=True)
    df_gait_joint.drop(df_gait_joint.index[350:567], inplace=True)

    df_gait_joint = df_gait_joint.reset_index(drop=True)

    first_step = df_gait_joint.iloc[0:72]
    full_step = df_gait_joint.iloc[73:182]
    full_step = full_step.reset_index(drop=True)
    step_close = df_gait_joint.iloc[246::]
    step_close = step_close.reset_index(drop=True)
    step_close = step_close.rename(columns={'LHFE': 'RHFE', 'LKFE': 'RKFE', 'RHFE':'LHFE', 'RKFE':'LKFE'})
    step_close = step_close[['LHAA', 'LHFE', 'LKFE', 'LADPF', 'RHAA', 'RHFE', 'RKFE', 'RADPF']]
    
    # plot_joints(first_step)
    # plot_joints(full_step)
    # plot_joints(step_close)

    return first_step, full_step, step_close, df_gait_joint

def no_shooting_approaching_stand(step_close, first_step):
    # print(step_close.shape)
    begin_point = step_close.iloc[-1]
    end_point = first_step.iloc[0]
    df2 = step_close.iloc[:0,:].copy()  
    print(df2)  
    for i in range(len(step_close.columns)):
        res = np.linspace(begin_point[i], end_point[i], 20)
        df2[step_close.columns[i]] = res 
    step_close = pd.concat([step_close, df2])
    return step_close

def step_close_update(step_close, full_step, first_step):

    plot_joints(step_close)

    res_lhfe = extract_max_min('LHFE', step_close, first_step, full_step)
    res_rhfe = extract_max_min('RHFE', step_close, first_step, full_step)
    res_lkfe = extract_max_min('LKFE', step_close, first_step, full_step)
    res_rkfe = extract_max_min('RKFE', step_close, first_step, full_step)


    interpolate_linear('LKFE', res_lkfe, step_close)
    interpolate_linear('RKFE', res_rkfe, step_close)
    interpolate_linear('LHFE', res_lhfe, step_close)
    interpolate_linear('RHFE', res_rhfe, step_close)

    plot_joints(step_close)

    return step_close

def extract_max_min(joint, step_close, first_step, full_step):
    local_max_joint = step_close[joint][(step_close[joint].shift(1) < step_close[joint]) & (step_close[joint].shift(-1) < step_close[joint])]
    local_min_joint= step_close[joint][(step_close[joint].shift(1) > step_close[joint]) & (step_close[joint].shift(-1) > step_close[joint])]
    res_joint = list(zip(local_max_joint,local_max_joint.index))
    res_joint += list(zip(local_min_joint, local_min_joint.index))
    res_joint.sort(key = lambda x: x[1])
    res_joint.append((first_step[joint].iloc[0], step_close[joint].index[-1]))
    res_joint.insert(0, (full_step[joint].iloc[-1], step_close[joint].index[0]))
    return res_joint

def interpolate_linear(joint, res_joint, step_close):
    x_joint = [x[1] for x in res_joint]
    y_joint = [x[0] for x in res_joint]
    joint_interp = interp1d(x_joint, y_joint)
    x_new_joint = np.linspace(res_joint[0][1], res_joint[-1][1], len(step_close))
    step_close[joint] = joint_interp(x_new_joint)

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
    time_points = 100
    lhaa = np.linspace(-0.06, -0.06, time_points)
    lhfe = np.linspace(0.351166, 1.5, time_points)
    lkfe = np.linspace(0.372236, 1.2, time_points)
    ladpf = np.linspace(0.119176, -0.5, time_points)

    rhaa = np.linspace(-0.06, -0.06, time_points)
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
    lhaa = np.linspace(-0.06, -0.06, time_points)
    lhfe = np.linspace(1.5, 0.351166, time_points)
    lkfe = np.linspace(1.2, 0.372236, time_points)
    ladpf = np.linspace(-0.5, 0.119176, time_points)

    rhaa = np.linspace(-0.06, -0.06, time_points)
    rhfe = np.linspace(1.5, 0.274982, time_points)
    rkfe = np.linspace(1.2, 0.286291, time_points)
    radpf = np.linspace(-0.5, 0.077083, time_points)

    sit_to_stand = np.column_stack([
        lhaa, lhfe, lkfe, ladpf, rhaa, rhfe, rkfe, radpf
    ])

    np.savetxt('./ros2/src/march_gait_planning/m9_gait_files/joint_angles/sit_to_stand.csv', 
               sit_to_stand, delimiter=',')
    
def hinge_gait():
    time_points = 100
    lhaa = np.linspace(-0.06, -0.06, time_points)
    lhfe = np.linspace(0.351166, 1.5, time_points)
    lkfe = np.linspace(0.372236, 0.372236, time_points)
    ladpf = np.linspace(0.119176, 0.119176, time_points)

    rhaa = np.linspace(-0.06, -0.06, time_points)
    rhfe = np.linspace(0.274982, 1.5, time_points)
    rkfe = np.linspace(0.286291, 0.286291, time_points)
    radpf = np.linspace(0.077083, 0.077083, time_points)


    hinge_gait = np.column_stack([
        lhaa, lhfe, lkfe, ladpf, rhaa, rhfe, rkfe, radpf
    ])

    np.savetxt('./ros2/src/march_gait_planning/m9_gait_files/joint_angles/hinge_gait.csv', 
               hinge_gait, delimiter=',')

def sideways(): 
    times = 100
    ladpf1 = np.linspace(0.119176, 0.119176, times)
    lhaa1 = np.linspace(-0.06, -0.1745, times)
    lhfe1 = np.linspace(0.351166, 0.351166, times)
    lkfe1 = np.linspace(0.372236, 0.5236, times)
    radpf1 = np.linspace(0.077083, 0.077083, times)
    rhaa1 = np.append(np.linspace(-0.06, -0.1, int(((2000000000-1500000000)/2000000000)*times)), np.linspace(-0.1, -0.1745, int(((1500000000)/2000000000)*times)))
    rhfe1 = np.linspace(0.274982, 0.274982, times)
    rkfe1 = np.linspace(0.286291, 0.5236, times)
    left_open = np.column_stack([
        lhaa1, lhfe1, lkfe1, ladpf1, rhaa1, rhfe1, rkfe1, radpf1
    ]) 

    ladpf = np.linspace(0.119176, 0.119176, times)
    lhaa = np.append(np.linspace(-0.1745, -0.1, int((1500000000/2000000000)*times)), np.linspace(-0.1, -0.06, int(((2000000000-1500000000)/2000000000)*times)))
    lhfe = np.linspace(0.351166, 0.351166, times)
    lkfe = np.linspace(0.5236, 0.372236, times)
    radpf = np.linspace(0.077083, 0.077083, times)
    rhaa = np.linspace(-0.1745, -0.06, times)
    rhfe = np.linspace(0.274982, 0.274982, times)
    rkfe = np.linspace(0.5236, 0.286291, times)
    right_close = np.column_stack([
        lhaa, lhfe, lkfe, ladpf, rhaa, rhfe, rkfe, radpf
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


    

first_step_def, one_step, step_close_def, full_gait = normal_large_step()
step_close_updated = no_shooting_approaching_stand(step_close_def, first_step_def)
# first_step_updated = pd.read_csv('./ros2/src/march_gait_planning/m9_gait_files/joint_angles/first_step_q.csv', names=['LHAA', 'LHFE', 'LKFE', 'LADPF', 'RHAA', 'RHFE', 'RKFE', 'RADPF'])
# one_step_updated = pd.read_csv('./ros2/src/march_gait_planning/m9_gait_files/joint_angles/one_step_q.csv', names=['LHAA', 'LHFE', 'LKFE', 'LADPF', 'RHAA', 'RHFE', 'RKFE', 'RADPF'])
# step_close_updated = step_close_update(step_close, one_step_updated, first_step_updated)

# first_step.to_csv('./ros2/src/march_gait_planning/m9_gait_files/joint_angles/first_step.csv', sep=',', header=False, index=False)
# one_step.to_csv('./ros2/src/march_gait_planning/m9_gait_files/joint_angles/one_step.csv', sep=',', header=False, index=False)
step_close_updated.to_csv('./ros2/src/march_gait_planning/m9_gait_files/joint_angles/step_close.csv', sep=',', header=False, index=False)

# one_step_updated = step_close_update(one_step_updated, one_step_updated, one_step_updated)
# one_step_updated.to_csv('./ros2/src/march_gait_planning/m9_gait_files/joint_angles/one_step_q_updated.csv', sep=',', header=False, index=False)

# first_step.to_csv('./ros2/src/march_gait_planning/m9_gait_files/joint_angles/first_step_q.csv', sep=',', header=False, index=False)
# one_step.to_csv('./ros2/src/march_gait_planning/m9_gait_files/joint_angles/one_step_q.csv', sep=',', header=False, index=False)
# step_close_updated.to_csv('./ros2/src/march_gait_planning/m9_gait_files/joint_angles/step_close_q.csv', sep=',', header=False, index=False)
# full_gait.to_csv('./ros2/src/march_gait_planning/m9_gait_files/joint_angles/full_gait_q.csv', sep=',', header=False, index=False)

hinge_gait()