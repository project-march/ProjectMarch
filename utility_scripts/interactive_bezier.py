import numpy as np
import matplotlib.pyplot as plt
import bezier 
import yaml 
import os
import tkinter as tk
from tkinter import messagebox
from utility_scripts.convert_bezier_points_to_csv import stair_gaits, descend_stairs
from scipy.interpolate import interp1d

LEG_LENGTH = 0.912

def make_evenly_spaced_points(x, z, array_size):
    # Create an array of cumulative distances between points
    distances = np.sqrt(np.diff(x)**2 + np.diff(z)**2)
    cumulative_distances = np.insert(np.cumsum(distances), 0, 0)

    # Generate new evenly spaced distances
    even_distances = np.linspace(cumulative_distances[0], cumulative_distances[-1], num=array_size)

    # Interpolate the x and z values over the new distances
    interp_x = interp1d(cumulative_distances, x, kind='cubic')
    interp_z = interp1d(cumulative_distances, z, kind='cubic')

    x_new = interp_x(even_distances)
    z_new = interp_z(even_distances)

    return x_new, z_new


def calculate_bezier_curve(points, array_size = 100):
    """Calculate the Bezier curve for the given points."""
    curve = bezier.Curve(points.T, degree=3)
    t_values = np.linspace(0.0, 1.0, array_size)
    curve_points = curve.evaluate_multi(t_values)
    return curve_points.T

def create_bezier_csv(points, array_size, gait_type):
    pause_time = int(array_size/6)
    step_length = points.max(axis=0)[0]
    ending_angle = np.arcsin((step_length/2)/LEG_LENGTH)

    # ------------------------------------------------- FIRST STEP ----------------------------------------------------------------------------------
    phi_first_step = np.linspace(0, ending_angle, int(array_size))

    # The stance leg follows a circle with radius LEG_LENGTH with center at (0, LEG_LENGTH). Sine and cosine are swapped from a usual circle, since we want the circle to start at the bottom and go backwards
    x_stance_first_step = -LEG_LENGTH*np.sin(phi_first_step)
    z_stance_first_step = LEG_LENGTH - LEG_LENGTH*np.cos(phi_first_step)

    x_stance_first_step = np.append(x_stance_first_step, x_stance_first_step[-1]*np.ones(pause_time))
    z_stance_first_step = np.append(z_stance_first_step, z_stance_first_step[-1]*np.ones(pause_time))

    vertical_offset = z_stance_first_step[-1]

    first_step_points = points.copy()
    first_step_points[:, 0] = first_step_points[:, 0]/2
    first_step_points[1:, 1] = first_step_points[1:, 1] + vertical_offset

    curvexz_first_step = bezier.Curve(first_step_points.T, degree=3)
    number_of_time_points_first_step = np.linspace(0, 1.0, int(array_size))
    points_first_step= curvexz_first_step.evaluate_multi(number_of_time_points_first_step)
    x_swing_first_step = points_first_step[0,:]
    z_swing_first_step = points_first_step[1,:]

    x_swing_first_step, z_swing_first_step = make_evenly_spaced_points(x_swing_first_step, z_swing_first_step, int(array_size))

    x_swing_first_step = np.append(x_swing_first_step, x_swing_first_step[-1]*np.ones(pause_time))
    z_swing_first_step = np.append(z_swing_first_step, z_swing_first_step[-1]*np.ones(pause_time))

    final_points_first_step = np.column_stack((x_swing_first_step, z_swing_first_step, x_stance_first_step, z_stance_first_step))

    plt.scatter(x_swing_first_step, z_swing_first_step)
    plt.plot(x_stance_first_step, z_stance_first_step, color="orange")
    plt.title("First Step")
    plt.show()

    # ------------------------------------------------- FULL STEP ----------------------------------------------------------------------------------
    phi_full_step = np.linspace(-ending_angle, ending_angle, int(array_size))

    x_stance_complete_step = -LEG_LENGTH*np.sin(phi_full_step)
    z_stance_complete_step = LEG_LENGTH - LEG_LENGTH*np.cos(phi_full_step)

    x_stance_complete_step = np.append(x_stance_complete_step, x_stance_complete_step[-1]*np.ones(pause_time))
    z_stance_complete_step = np.append(z_stance_complete_step, z_stance_complete_step[-1]*np.ones(pause_time))

    full_step_points = points.copy()
    full_step_points[:, 1] = full_step_points[:, 1] + vertical_offset

    curvexz_complete_step = bezier.Curve(full_step_points.T, degree=3)
    number_of_time_points_complete_step = np.linspace(0, 1.0, array_size)
    points_complete_step= curvexz_complete_step.evaluate_multi(number_of_time_points_complete_step)
    x_swing_complete_step = points_complete_step[0,:] - (step_length/2)
    z_swing_complete_step = points_complete_step[1,:]
    
    x_swing_complete_step, z_swing_complete_step = make_evenly_spaced_points(x_swing_complete_step, z_swing_complete_step, array_size)
    
    x_swing_complete_step = np.append(x_swing_complete_step, x_swing_complete_step[-1]*np.ones(pause_time))
    z_swing_complete_step = np.append(z_swing_complete_step, z_swing_complete_step[-1]*np.ones(pause_time))

    final_points_complete_step = np.column_stack((x_swing_complete_step, z_swing_complete_step, x_stance_complete_step, z_stance_complete_step))

    plt.plot(x_swing_complete_step, z_swing_complete_step)
    plt.plot(x_stance_complete_step, z_stance_complete_step, color="orange")
    plt.title("Full Step")
    plt.show()

    # ------------------------------------------------- STEP CLOSE ----------------------------------------------------------------------------------
    phi_step_close = np.linspace(0, -ending_angle, int(array_size))

    x_stance_step_close = -LEG_LENGTH*np.sin(phi_step_close)
    z_stance_step_close = LEG_LENGTH - LEG_LENGTH*np.cos(phi_step_close)

    # x_stance_step_close = np.append(x_stance_step_close, x_stance_step_close[-1]*np.ones(pause_time))
    # z_stance_step_close = np.append(z_stance_step_close, z_stance_step_close[-1]*np.ones(pause_time))

    step_close_points = points.copy()
    step_close_points[:, 0] = step_close_points[:, 0]/-2
    step_close_points[1:, 1] = step_close_points[1:, 1] + vertical_offset
    curvexz_step_close = bezier.Curve(step_close_points.T, degree=3)
    number_of_time_points_step_close = np.linspace(0, 1.0, int(array_size))
    points_step_close= curvexz_step_close.evaluate_multi(number_of_time_points_step_close)
    x_swing_step_close = points_step_close[0,:]
    z_swing_step_close = points_step_close[1,:]

    x_swing_step_close, z_swing_step_close = make_evenly_spaced_points(x_swing_step_close, z_swing_step_close, array_size)

    # x_swing_step_close = np.append(x_swing_step_close, x_swing_step_close[-1]*np.ones(pause_time))
    # z_swing_step_close = np.append(z_swing_step_close, z_swing_step_close[-1]*np.ones(pause_time))

    final_points_step_close = np.column_stack((x_swing_step_close, z_swing_step_close, x_stance_step_close, z_stance_step_close))
    final_points_step_close = np.flip(final_points_step_close, axis=0)

    plt.plot(x_swing_step_close, z_swing_step_close)
    plt.plot(x_stance_step_close, z_stance_step_close, color="orange")
    plt.title("Full Step")
    plt.show()

    if gait_type == "large_gait":
        np.savetxt('ros2/src/march_gait_planning/m9_gait_files/cartesian/first_step_large.csv', final_points_first_step, delimiter=',')
        np.savetxt('ros2/src/march_gait_planning/m9_gait_files/cartesian/normal_gait_large.csv', final_points_complete_step, delimiter=',')
        np.savetxt('ros2/src/march_gait_planning/m9_gait_files/cartesian/large_step_close.csv', final_points_step_close, delimiter=',')
    elif gait_type == "small_gait":
        np.savetxt('ros2/src/march_gait_planning/m9_gait_files/cartesian/first_step_small.csv', final_points_first_step, delimiter=',')
        np.savetxt('ros2/src/march_gait_planning/m9_gait_files/cartesian/normal_gait_small.csv', final_points_complete_step, delimiter=',')
        np.savetxt('ros2/src/march_gait_planning/m9_gait_files/cartesian/small_step_close.csv', final_points_step_close, delimiter=',')

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
            create_bezier_csv(self.points, self.array_size, self.gait_type)
            self.updated_points = self.points.tolist()
        else:
            self.updated_points = self.initial_points.tolist()

def interactive_bezier(gait_type: str, array_size: int): 
    if gait_type == "small_gait" or gait_type == "large_gait":  
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
            interactive_bezier = InteractiveBezier(points, gait_type)
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
            

    with open('utility_scripts/points.yaml', 'w') as f:
            yaml.dump(data, f)
    
    if gait_type == "ascending":
        ascend_csv = stair_gaits(data['ascending'], array_size)
        descend_csv = descend_stairs(ascend_csv)
        np.savetxt('ros2/src/march_gait_planning/m9_gait_files/cartesian/ascend_test.csv', ascend_csv, delimiter=',')
        np.savetxt('ros2/src/march_gait_planning/m9_gait_files/cartesian/descend_test.csv', descend_csv, delimiter=',')



# interactive_bezier("small_gait", 600)
interactive_bezier("large_gait", 600)