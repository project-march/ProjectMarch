import numpy as np
import matplotlib.pyplot as plt
import bezier 
import yaml 
import os
import tkinter as tk
from tkinter import messagebox

def calculate_bezier_curve(points):
    """Calculate the Bezier curve for the given points."""
    curve = bezier.Curve(points.T, degree=3)
    t_values = np.linspace(0.0, 1.0, 100)
    curve_points = curve.evaluate_multi(t_values)
    return curve_points.T

def create_bezier_csv(points, array_size, gait_type):
    step_length = points.max(axis=0)[0]

    first_step_points = points.copy()
    first_step_points[:, 0] = first_step_points[:, 0]/2
    curvexz_first_step = bezier.Curve(first_step_points.T, degree=3)
    number_of_time_points_first_step = np.linspace(0, 1.0, int(array_size/2))
    points_first_step= curvexz_first_step.evaluate_multi(number_of_time_points_first_step)
    x_swing_first_step = points_first_step[0,:]
    z_swing_first_step = points_first_step[1,:]
    x_stance_first_step = np.linspace(0, 0 - (step_length/2), int(array_size/2))
    # z_stance_first_step = compensation_for_circle(int(array_size/2), step_length/2)
    z_stance_first_step = [0]*int(array_size/2)
    # z_swing_first_step = z_swing_first_step + z_stance_first_step
    final_points_first_step = np.column_stack((x_swing_first_step, z_swing_first_step, x_stance_first_step, z_stance_first_step))

    curvexz_complete_step = bezier.Curve(points.T, degree=3)
    print(points.T)
    number_of_time_points_complete_step = np.linspace(0, 1.0, array_size)
    points_complete_step= curvexz_complete_step.evaluate_multi(number_of_time_points_complete_step)
    x_swing_complete_step = points_complete_step[0,:] - (step_length/2)
    z_swing_complete_step = points_complete_step[1,:]
    x_stance_complete_step = np.linspace(0+(step_length/2), 0 - (step_length/2), array_size)
    # z_stance_complete_step = compensation_for_circle(array_size, step_length)
    z_stance_complete_step = [0]*array_size
    # z_swing_complete_step = z_swing_complete_step + z_stance_complete_step
    final_points_complete_step = np.column_stack((x_swing_complete_step, z_swing_complete_step, x_stance_complete_step, z_stance_complete_step))

    if gait_type == "large_gait":
        np.savetxt('ros2/src/march_gait_planning/m9_gait_files/cartesian/first_step_large.csv', final_points_first_step, delimiter=',')
        np.savetxt('ros2/src/march_gait_planning/m9_gait_files/cartesian/normal_gait_large.csv', final_points_complete_step, delimiter=',')
    elif gait_type == "small_gait":
        np.savetxt('ros2/src/march_gait_planning/m9_gait_files/cartesian/first_step_small.csv', final_points_first_step, delimiter=',')
        np.savetxt('ros2/src/march_gait_planning/m9_gait_files/cartesian/normal_gait_small.csv', final_points_complete_step, delimiter=',')


class DraggablePoint:
    """A class to represent a draggable point in matplotlib."""
    def __init__(self, parent, point):
        self.parent = parent
        self.point = point
        self.artist = plt.Circle((point[0], point[1]), 0.01, fc='r')
        self.artist.set_picker(5)  # Tolerance for picking up the point

    def on_pick(self, event):
        """Handle the pick event."""
        if event.artist == self.artist:
            self.parent.dragging = self

    def on_release(self, event):
        """Handle the release event."""
        self.parent.dragging = None

    def on_motion(self, event):
        """Handle the motion event."""
        if self.parent.dragging is not self:
            return
        self.point[0] = event.xdata
        self.point[1] = event.ydata
        self.artist.center = self.point[0], self.point[1]
        self.parent.update()
    
    def update_position(self):
        """Update the position of the point."""
        self.artist.center = self.point[0], self.point[1]

class InteractiveBezier:
    """A class to represent an interactive Bezier curve in matplotlib."""
    def __init__(self, data, gait_type):
        self.gait_type = gait_type
        self.data = data
        self.points = np.array(data[self.gait_type])
        self.draggable_points = [DraggablePoint(self, point) for point in self.points]
        self.dragging = None
        self.initial_points = self.points.copy()
        curve_points = self.calculate_bezier_curve()
        self.line, = plt.plot(curve_points[:, 0], curve_points[:, 1])

        # Create a separate window for the coordinates
        self.root = tk.Tk()
        self.root.title('Coordinates')

        # Create labels for the columns
        tk.Label(self.root, text='Point').grid(row=0, column=0)
        tk.Label(self.root, text='X').grid(row=0, column=1)
        tk.Label(self.root, text='Y').grid(row=0, column=2)

        # Create entry fields for the coordinates
        self.entries = []
        for i, point in enumerate(self.points):
            tk.Label(self.root, text=f'{i + 1}').grid(row=i + 1, column=0)
            for j in range(2):
                entry = tk.Entry(self.root)
                entry.insert(0, str(point[j]))
                entry.grid(row=i + 1, column=j + 1)
                self.entries.append(entry)

        # Create a button to update the points
        tk.Button(self.root, text='Update points', command=self.update_points).grid(row=len(self.points) + 1, column=0, columnspan=3)
        # Bind the closing of the plot window to a function
        plt.get_current_fig_manager().canvas.figure.canvas.mpl_connect('close_event', self.on_close)

        self.update()


    def calculate_bezier_curve(self):
        """Calculate the Bezier curve for the current points."""
        return calculate_bezier_curve(self.points)
    
    def update_points(self):
        """Update the points based on the entries."""
        for i, entry in enumerate(self.entries):
            self.points[i // 2][i % 2] = float(entry.get())
            # Update the position of the draggable point
            self.draggable_points[i // 2].update_position()
        self.update()

    def update(self):
        """Update the plot."""
        curve_points = self.calculate_bezier_curve()
        self.line.set_data(curve_points[:, 0], curve_points[:, 1])
            # Calculate the limits of the axes
        x_min, y_min = self.points.min(axis=0)
        x_max, y_max = self.points.max(axis=0)
        print(x_min, y_min, x_max, y_max)
        # Set the limits of the axes with a 10% margin
        plt.xlim(x_min - 0.1 * (x_max - x_min), x_max + 0.1 * (x_max - x_min))
        plt.ylim(y_min - 0.1 * (y_max - y_min), y_max + 0.1 * (y_max - y_min))

        plt.draw()
        
        # Update the entries
        for i, entry in enumerate(self.entries):
            entry.delete(0, tk.END)
            entry.insert(0, str(self.points[i // 2][i % 2]))

    def connect(self):
        """Connect the draggable points to the plot."""
        for point in self.draggable_points:
            plt.gca().add_patch(point.artist)
        plt.gcf().canvas.mpl_connect('pick_event', self.on_pick)
        plt.gcf().canvas.mpl_connect('button_release_event', self.on_release)
        plt.gcf().canvas.mpl_connect('motion_notify_event', self.on_motion)

    def on_pick(self, event):
        """Handle the pick event."""
        for point in self.draggable_points:
            point.on_pick(event)

    def on_release(self, event):
        """Handle the release event."""
        for point in self.draggable_points:
            point.on_release(event)

    def on_motion(self, event):
        """Handle the motion event."""
        for point in self.draggable_points:
            point.on_motion(event)
    
    def on_close(self, event):
        """Handle the close event."""
        root = tk.Tk()
        root.withdraw()

        # Check if the last point has been altered
        if not np.array_equal(self.points[-1], self.initial_points[-1]):
            # Ask whether to change the step size
            if not messagebox.askyesno('Change step size', 'You are changing the step size, are you sure?'):
                # If the user clicks 'No', return without saving the points
                root.destroy()
                # Also destroy the coordinates window
                self.root.destroy()
                return
        # Destroy the root window
        root.destroy()
        # Also destroy the coordinates window
        self.root.destroy() 
        # Ask whether to save the points
        if messagebox.askyesno('Save points', 'Do you want to save the points?'):
            # Save the points to a file
            self.data[self.gait_type] = self.points.tolist()
            with open('utility_scripts/points.yaml', 'w') as f:
                yaml.dump(self.data, f)
            create_bezier_csv(self.points, 200, self.gait_type)


# Load the points from the file if it exists, otherwise use default points
if os.path.exists('utility_scripts/points.yaml'):
    with open('utility_scripts/points.yaml', 'r') as f:
        data = yaml.safe_load(f)
else:
    print("No points file found")

interactive_bezier = InteractiveBezier(data, "small_gait")
interactive_bezier.connect()

# Connect the on_close function to the close event
plt.gcf().canvas.mpl_connect('close_event', interactive_bezier.on_close)
plt.grid()

# Start the matplotlib event loop
plt.show()
