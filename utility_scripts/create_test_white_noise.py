import csv
import random

def generate_white_noise(amplitude, duration, sample_rate):
    num_samples = int(duration * sample_rate)
    samples = [random.uniform(-amplitude, amplitude) for _ in range(num_samples)]
    return samples

def save_to_csv(filename, data):
    with open(filename, 'w', newline='') as file:
        writer = csv.writer(file)
        for i, amplitude in enumerate(data):
            time = i / sample_rate
            writer.writerow([amplitude])

amplitude = 0.3  # Adjust the amplitude as needed
duration = 5.0  # Adjust the duration in seconds
sample_rate = 50 # Adjust the sample rate as needed

data = generate_white_noise(amplitude, duration, sample_rate)
filename = 'ros2/src/march_gait_planning/m9_gait_files/white_noise_gait.csv'  # Replace with your desired file path
save_to_csv(filename, data)
