import csv
import random

def generate_white_noise(amplitude, duration, sample_rate):
    num_samples = int(duration * sample_rate)
    samples = [random.uniform(-amplitude, amplitude) for _ in range(num_samples)]
    return samples

def save_to_csv(filename, data):
    with open(filename, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['Time', 'Amplitude'])
        for i, amplitude in enumerate(data):
            time = i / sample_rate
            writer.writerow([time, amplitude])

amplitude = 0.5  # Adjust the amplitude as needed
duration = 10.0  # Adjust the duration in seconds
sample_rate = 44100  # Adjust the sample rate as needed

data = generate_white_noise(amplitude, duration, sample_rate)
filename = '/home/mick/march/ros2/src/march_gait_planning/m9_gait_files/static_white_noise.csv'  # Replace with your desired file path
save_to_csv(filename, data)
