import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Slider, Button
from march_goniometric_ik_solver.ik_solver_v2 import Pose

# Create zero pose:
pose = Pose()
pose.solve_all(0.0, 0.0, 0.5)
positions = pose.calculate_joint_positions()
positions_x = [pos[0] for pos in positions]
positions_y = [pos[1] for pos in positions]

# Create the figure and the line that we will manipulate
fig, ax = plt.subplots()
(exo,) = plt.plot(positions_x, positions_y, ".-")

# adjust the main plot to make room for the sliders
plt.subplots_adjust(left=0.25, bottom=0.25)
plt.ylim(-0.4, 0.9)
plt.xlim(-0.1, 1.2)
plt.gca().set_aspect("equal", adjustable="box")
plt.grid()

# Make a horizontal slider to control ankle_x:
axfreq = plt.axes([0.25, 0.1, 0.65, 0.03])
x_slider = Slider(
    ax=axfreq,
    label="anle_x",
    valmin=0.0,
    valmax=0.6,
    valinit=0.0,
)

# Make a vertically oriented slider to control ankle_y
axamp = plt.axes([0.2, 0.2, 0.0225, 0.63])
y_slider = Slider(
    ax=axamp,
    label="ankle_y",
    valmin=-0.3,
    valmax=0.3,
    valinit=0.0,
    orientation="vertical",
)

# Make a vertically oriented slider to control hip_fraction
axfreq = plt.axes([0.25, 0.9, 0.65, 0.03])
hip_slider = Slider(
    ax=axfreq,
    label="hip_fraction",
    valmin=0.0,
    valmax=1.0,
    valinit=0.5,
)

# Make a vertically oriented slider to control knee_bed
axfreq = plt.axes([0.25, 0.95, 0.65, 0.03])
knee_slider = Slider(
    ax=axfreq,
    label="knee_bend",
    valmin=0.1,
    valmax=10,
    valinit=8,
)


# The function to be called anytime a slider's value changes
def update(update_value):
    pose = Pose()
    pose.solve_all(
        x_slider.val, y_slider.val, hip_slider.val, np.deg2rad(knee_slider.val)
    )
    positions = pose.calculate_joint_positions()
    positions_x = [pos[0] for pos in positions]
    positions_y = [pos[1] for pos in positions]

    exo.set_xdata(positions_x)
    exo.set_ydata(positions_y)
    fig.canvas.draw_idle()


# register the update function with each slider
x_slider.on_changed(update)
y_slider.on_changed(update)
hip_slider.on_changed(update)
knee_slider.on_changed(update)

# Create a `matplotlib.widgets.Button` to reset the sliders to initial values.
resetax = plt.axes([0.8, 0.025, 0.1, 0.04])
button = Button(resetax, "Reset", hovercolor="0.975")


def reset(event):
    x_slider.reset()
    y_slider.reset()
    hip_slider.reset()
    knee_slider.reset()


button.on_clicked(reset)

plt.show()
