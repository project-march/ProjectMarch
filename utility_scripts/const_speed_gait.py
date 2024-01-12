import csv
import random

noise_amplintude = 0.3
sample_rate = 60 # Hz

RoM_lb = -0.536 # radians
RoM_ub = 1.058 # radians
range_of_motion = RoM_ub - RoM_lb # radians
zero_offset = 0.0 # radians
velocity = 0.1 # radians per second

def generate_const_vel(range_of_motion, zero_offset,velocity, sample_rate):
    num_samples = int(range_of_motion/velocity*sample_rate)
    step = range_of_motion/num_samples
    samples = [zero_offset + step*i for i in range(num_samples+1)]
    return samples

def save_to_csv(filename, data):
    with open(filename, 'w', newline='') as file:
        writer = csv.writer(file)
        for i, amplitude in enumerate(data):
            time = i / sample_rate
            writer.writerow([amplitude])

data = generate_const_vel(range_of_motion, zero_offset,velocity, sample_rate)
filename = 'ros2/src/march_gait_planning/m9_gait_files/const_vel_gait.csv'  # Replace with your desired file path
save_to_csv(filename, data)
