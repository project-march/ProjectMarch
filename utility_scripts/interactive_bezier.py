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


def create_bezier_csv(points, array_size):
    step_length = points.max(axis=0)[0]
    curvexz_complete_step = bezier.Curve(points.T, degree=3)
    number_of_time_points_complete_step = np.linspace(0, 1.0, array_size)
    points_complete_step= curvexz_complete_step.evaluate_multi(number_of_time_points_complete_step)
    x_swing_complete_step = points_complete_step[0,:] - (step_length/2)
    z_swing_complete_step = points_complete_step[1,:]
    x_stance_complete_step = np.linspace(0+(step_length/2), 0 - (step_length/2), array_size)
    # z_stance_complete_step = compensation_for_circle(array_size, step_length)
    z_stance_complete_step = [0]*array_size
    # z_swing_complete_step = z_swing_complete_step + z_stance_complete_step
    final_points_complete_step = np.column_stack((x_swing_complete_step, z_swing_complete_step, x_stance_complete_step, z_stance_complete_step))

    np.savetxt('ros2/src/march_gait_planning/m9_gait_files/cartesian/normal_gait_large.csv', final_points_complete_step, delimiter=',')


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

class InteractiveBezier:
    """A class to represent an interactive Bezier curve in matplotlib."""
    def __init__(self, points):
        self.points = points
        self.draggable_points = [DraggablePoint(self, point) for point in points]
        self.dragging = None
        self.initial_points = points.copy()
        curve_points = self.calculate_bezier_curve()
        self.line, = plt.plot(curve_points[:, 0], curve_points[:, 1])
        self.update()

    def calculate_bezier_curve(self):
        """Calculate the Bezier curve for the current points."""
        return calculate_bezier_curve(self.points)

    def update(self):
        """Update the plot."""
        curve_points = self.calculate_bezier_curve()
        self.line.set_data(curve_points[:, 0], curve_points[:, 1])
            # Calculate the limits of the axes
        x_min, y_min = points.min(axis=0)
        x_max, y_max = points.max(axis=0)
        print(x_min, y_min, x_max, y_max)
        # Set the limits of the axes with a 10% margin
        plt.xlim(x_min - 0.1 * (x_max - x_min), x_max + 0.1 * (x_max - x_min))
        plt.ylim(y_min - 0.1 * (y_max - y_min), y_max + 0.1 * (y_max - y_min))

        plt.draw()

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
                return

        # Ask whether to save the points
        if messagebox.askyesno('Save points', 'Do you want to save the points?'):
            # Save the points to a file
            with open('utility_scripts/points.yaml', 'w') as f:
                yaml.dump(self.points.tolist(), f)
            create_bezier_csv(self.points, 200)

        # Destroy the root window
        root.destroy()      

# Initialize the plot
# plt.axis([0, 5, 0, 5])

# Create the interactive Bezier curve and connect it to the plot

# Load the points from the file if it exists, otherwise use default points
if os.path.exists('utility_scripts/points.yaml'):
    with open('utility_scripts/points.yaml', 'r') as f:
        points = np.array(yaml.safe_load(f))
else:
    print("No points file found")

interactive_bezier = InteractiveBezier(points)
interactive_bezier.connect()

# Connect the on_close function to the close event
plt.gcf().canvas.mpl_connect('close_event', interactive_bezier.on_close)
plt.grid()

# Start the matplotlib event loop
plt.show()
