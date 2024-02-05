import pandas as pd 
import matplotlib.pyplot as plt 
import numpy as np

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

    max_rhfe = df_gait_joint[['RHFE']].idxmax()
    max_rhfe_second = df_gait_joint[['RHFE']].iloc[150:250].idxmax()

    first_step = df_gait_joint.iloc[0:72]
    # first_step.to_csv('./ros2/src/march_gait_planning/m9_gait_files/joint_angles/first_step_q.csv', sep=',', header=False, index=False)
    full_step = df_gait_joint.iloc[73:182]
    # full_step.to_csv('./ros2/src/march_gait_planning/m9_gait_files/joint_angles/one_step_q.csv', sep=',', header=False, index=False)
    step_close = df_gait_joint.iloc[183::]
    # step_close.to_csv('./ros2/src/march_gait_planning/m9_gait_files/joint_angles/step_close_q.csv', sep=',', header=False, index=False)

    # plot_joints(first_step)
    plot_joints(full_step)

    df_gait_joint.to_csv('./ros2/src/march_gait_planning/m9_gait_files/joint_angles/full_gait_q.csv', sep=',', header=False, index=False)

def step_close():
    df_first_step = pd.read_csv('./ros2/src/march_gait_planning/m9_gait_files/joint_angles/first_step_q.csv', names=['LHAA','LHFE', 'LKFE', 'LADPF', 'RHAA', 'RHFE', 'RKFE', 'RADPF'])
    df_full_step = pd.read_csv('./ros2/src/march_gait_planning/m9_gait_files/joint_angles/one_step_q.csv', names=['LHAA','LHFE', 'LKFE', 'LADPF', 'RHAA', 'RHFE', 'RKFE', 'RADPF'])
    df_gait_joint = pd.read_csv('./ros2/src/march_gait_planning/m9_gait_files/joint_angles/q_test.csv')
    df_gait_joint['LKFE'] = df_gait_joint['LKFE']*-1
    df_gait_joint['RKFE'] = df_gait_joint['RKFE']*-1
    df_gait_joint.drop(df_gait_joint.index[0:50], inplace=True)
    df_gait_joint.drop(df_gait_joint.index[350:567], inplace=True)
    df_gait_joint = df_gait_joint.reset_index(drop=True)
    step_close = df_gait_joint.iloc[183:350]
    step_close = step_close.reset_index(drop=True)
    # print("Full step final position: ",df_full_step.iloc[[-1]])
    # print("First step final position: \n",df_first_step.iloc[[-1]], '\n')
    # print("First step initial position: \n",df_first_step.iloc[[0]], '\n')
    # step_close = df_first_step
    # step_close = step_close.rename(columns={"LHFE": "RHFE", "LKFE": "RKFE", "RHFE": "LHFE", "RKFE": "LKFE"})
    # step_close['LHFE'] = step_close["LHFE"] + (0.194641 - 0.274982)
    # step_close['RHFE'] = step_close['RHFE'] + (0.722718 - 0.351166)
    # step_close['LKFE'] = step_close['LKFE'] + (0.450064 - 0.286291)
    # step_close['RKFE'] = step_close["RKFE"] + (0.302273 - 0.372236)
    # print("Step close final position: \n", step_close.iloc[[-1]])
    plot_joints(df_first_step)
    plot_joints(df_gait_joint)
    plot_joints(df_full_step)
    plot_joints(step_close)
    print("full step final position: \n", step_close.iloc[[-1]], "\n")
    print("step close first position: \n", step_close.iloc[[0]], "\n")
    print("Step Close final position: \n",step_close.iloc[[-1]], '\n')
    # step_close.to_csv('./ros2/src/march_gait_planning/m9_gait_files/joint_angles/step_close_q.csv', sep=',', header=False, index=False)

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


    
normal_large_step()
