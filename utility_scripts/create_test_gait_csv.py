import csv
import math
import numpy as np

total_time = 10  # seconds
step_time = 0.05  # seconds
total_steps = int(total_time / step_time)
timesteps = np.linspace(0, 2*math.pi, total_steps)

# The sinusoidal functions are designed to be inside the software limits while still being quite large.
# The equations are absolutely horrendous, so if you want to visualize them, use desmos or something.

with open('src/march_gait_planning/m9_gait_files/test_trajectory_adpf.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    # writer.writerow(["Time", "Value"])  # Write header

    for timestep in timesteps:
        value = 0.36* np.sin(timestep + 0.15056) - 0.07
        writer.writerow([value])

with open('src/march_gait_planning/m9_gait_files/test_trajectory_haa.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    # writer.writerow(["Time", "Value"])  # Write header

    for timestep in timesteps:
        value = -0.2225 * np.cos(timestep - 1.83219) - 0.0875
        writer.writerow([value])

total_time = 20
total_steps = int(total_time / step_time)
timesteps = np.linspace(0, 2*math.pi, total_steps)

with open('src/march_gait_planning/m9_gait_files/test_trajectory_hfe.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    # writer.writerow(["Time", "Value"])  # Write header

    for timestep in timesteps:
        value = -1.08* math.sin(timestep + (3 * math.pi)/2 -2.37513)+0.82
        writer.writerow([value])

with open('src/march_gait_planning/m9_gait_files/test_trajectory_kfe.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    # writer.writerow(["Time", "Value"])  # Write header

    for timestep in timesteps:
        value = 0.975*np.sin(timestep -1.42745)+1.025
        writer.writerow([value])