import pandas as pd
import matplotlib.pyplot as plt

def plot_csv_data(file_path):
    # Read the CSV file into a pandas DataFrame
    df = pd.read_csv(file_path)

    # Plot the data
    df.plot()

    # Show the graph
    plt.show()

plot_csv_data('/home/mick/march/ros2/src/march_gait_planning/m9_gait_files/static_white_noise.csv')