import csv
import math

total_time = 5  # seconds
step_time = 0.05  # seconds
total_steps = int(total_time / step_time)

with open('src/march_gait_planning/m9_gait_files/test_trajectory.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    # writer.writerow(["Time", "Value"])  # Write header

    for step in range(total_steps):
        time = step * step_time
        value = math.sin(2 * math.pi * (step / total_steps))
        writer.writerow([value])