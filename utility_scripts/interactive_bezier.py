import numpy as np
import matplotlib.pyplot as plt
import bezier 
import yaml 
import os
import tkinter as tk
from tkinter import messagebox
from utility_scripts.convert_bezier_points_to_csv import stair_gaits, descend_stairs, create_bezier_csv, create_high_step_csv
from scipy.interpolate import interp1d

LEG_LENGTH = 0.912
HIGH_LEVEL_FREQUENCY = 200

def calculate_bezier_curve(points, array_size = 100):
    """Calculate the Bezier curve for the given points."""
    curve = bezier.Curve(points.T, degree=3)
    t_values = np.linspace(0.0, 1.0, array_size)
    curve_points = curve.evaluate_multi(t_values)
    return curve_points.T

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
    def __init__(self, points, gait_type, array_size):
        self.gait_type = gait_type
        self.points = points
        self.draggable_points = [DraggablePoint(self, point) for point in self.points]
        self.dragging = None
        self.initial_points = self.points.copy()
        curve_points = self.calculate_bezier_curve()
        self.line, = plt.plot(curve_points[:, 0], curve_points[:, 1])
        self.updated_points = None
        self.array_size = array_size    

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
                self.updated_points = self.initial_points.tolist()
        # Destroy the root window
        root.destroy()
        # Also destroy the coordinates window
        self.root.destroy() 
        # Ask whether to save the points
        if messagebox.askyesno('Save points', 'Do you want to save the points?'):
            if self.gait_type == "small_gait" or self.gait_type == "large_gait":
                create_bezier_csv(self.points, self.array_size, self.gait_type)
            elif self.gait_type == "high_step_1" or  self.gait_type == "high_step_2" or  self.gait_type == "high_step_3":
                create_high_step_csv(self.points, self.gait_type, self.array_size)
            self.updated_points = self.points.tolist()
        else:
            self.updated_points = self.initial_points.tolist()

def interactive_bezier(gait_type: str, array_size: int): 
    if gait_type == "small_gait" or gait_type == "large_gait":  
        print("Opening File")
        if os.path.exists('utility_scripts/points.yaml'):
            with open('utility_scripts/points.yaml', 'r') as f:
                data = yaml.safe_load(f)
                points = np.array(data[gait_type])
        else:
            print("No points file found")
        
        print("Passing points to class")
        interactive_bezier = InteractiveBezier(points, gait_type, array_size)
        interactive_bezier.connect()

        # Connect the on_close function to the close event
        plt.gcf().canvas.mpl_connect('close_event', interactive_bezier.on_close)
        plt.grid()

        # Start the matplotlib event loop
        plt.show()

        new_points = interactive_bezier.updated_points
        data[gait_type] = new_points


        
    
    elif gait_type == "ascending":
        if os.path.exists('utility_scripts/points.yaml'):
            with open('utility_scripts/points.yaml', 'r') as f:
                data = yaml.safe_load(f)
        else:
            print("No points file found")

        for i, step_type_dict in enumerate(data[gait_type]):
            # Get the step type and the points
            step_type = list(step_type_dict.keys())[0]
            points = np.array(step_type_dict[step_type])

            # Pass the points to the InteractiveBezier class
            interactive_bezier = InteractiveBezier(points, gait_type, array_size)
            interactive_bezier.connect()

            # Connect the on_close function to the close event
            plt.gcf().canvas.mpl_connect('close_event', interactive_bezier.on_close)
            plt.grid()

            # Start the matplotlib event loop
            plt.show()

            # Get the new points
            new_points = interactive_bezier.updated_points

            # Store the new points in the same spot in the data variable
            data[gait_type][i][step_type] = new_points

    elif gait_type == "high_step_1" or  gait_type == "high_step_2" or  gait_type == "high_step_3":
        if os.path.exists('utility_scripts/points.yaml'):
            with open('utility_scripts/points.yaml', 'r') as f:
                data = yaml.safe_load(f)
                points = np.array(data[gait_type])
        else:
            print("No points file found")

        interactive_bezier = InteractiveBezier(points, gait_type, array_size)
        interactive_bezier.connect()

        # Connect the on_close function to the close event
        plt.gcf().canvas.mpl_connect('close_event', interactive_bezier.on_close)
        plt.grid()

        # Start the matplotlib event loop
        plt.show()

        new_points = interactive_bezier.updated_points
        data[gait_type] = new_points
        
            

    with open('utility_scripts/points.yaml', 'w') as f:
            yaml.dump(data, f)
    
    if gait_type == "ascending":
        ascend_csv = stair_gaits(data['ascending'], array_size)
        descend_csv = descend_stairs(ascend_csv)
        np.savetxt('ros2/src/march_gait_planning/m9_gait_files/cartesian/ascend_test.csv', ascend_csv, delimiter=',')
        np.savetxt('ros2/src/march_gait_planning/m9_gait_files/cartesian/descend_test.csv', descend_csv, delimiter=',')

step_time = 3 # seconds

# interactive_bezier("small_gait", 150)
interactive_bezier("high_step_3", step_time*HIGH_LEVEL_FREQUENCY)
