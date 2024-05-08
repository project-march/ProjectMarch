import csv
import math

total_time = 10  # seconds
step_time = 0.05  # seconds
total_steps = int(total_time / step_time)

with open('src/march_gait_planning/m9_gait_files/test_trajectory_adpf.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    # writer.writerow(["Time", "Value"])  # Write header

    for step in range(total_steps):
        time = step * step_time
        value = 0.1* math.sin(2 * math.pi * (step / total_steps))
        writer.writerow([value])

with open('src/march_gait_planning/m9_gait_files/test_trajectory_haa.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    # writer.writerow(["Time", "Value"])  # Write header

    for step in range(total_steps):
        time = step * step_time
        value = -0.05 * math.sin(2 * math.pi * (step / total_steps))
        writer.writerow([value])

with open('src/march_gait_planning/m9_gait_files/test_trajectory_hfe.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    # writer.writerow(["Time", "Value"])  # Write header

    for step in range(total_steps):
        time = step * step_time
        value = 0.1* math.sin(2 * math.pi * (step / total_steps))
        writer.writerow([value])

with open('src/march_gait_planning/m9_gait_files/test_trajectory_kfe.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    # writer.writerow(["Time", "Value"])  # Write header

    for step in range(total_steps):
        time = step * step_time
        value = 0.1* math.cos(2 * math.pi * (step / total_steps) + math.pi) + 0.1
        writer.writerow([value])