import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Slider, Button
from march_goniometric_ik_solver.ik_solver_v2 import Pose, LENGTH_FOOT


class LiveWidget:
    def __init__(self) -> None:
        self.default_hip_fraction = 0.5
        self.default_knee_bend = np.deg2rad(8)
        self.reduce_df_rear = False
        self.reduce_df_front = False

        # Create default pose:
        pose = Pose()
        pose.solve_all(
            0.0,
            0.0,
            self.default_hip_fraction,
            self.default_knee_bend,
            self.reduce_df_front,
            self.reduce_df_rear,
        )
        positions = pose.calculate_joint_positions()
        positions_x = [pos[0] for pos in positions]
        positions_y = [pos[1] for pos in positions]

        # Plot the default pose:
        self.fig, self.ax = plt.subplots()
        (self.exo,) = plt.plot(positions_x, positions_y, ".-")

        # Plot ankle goal and toes:
        (self.goal,) = plt.plot(0.0, 0.0, "x")
        (self.toes,) = plt.plot(LENGTH_FOOT, 0.0, "x")

        # Print the default pose:
        joints = [
            "ankle1",
            "hip1_aa",
            "hip1_fe",
            "knee1",
            "ankle2",
            "hip2_aa",
            "hip2_fe",
            "knee2",
        ]
        pose_rad = pose.pose_left
        pose_deg = [np.rad2deg(angle) for angle in pose_rad]
        celltext = np.column_stack(
            (joints, np.round(pose_rad, 2), np.round(pose_deg, 2))
        )
        collabel = ("joint", "radians", "degrees")
        self.table = plt.table(cellText=celltext, colLabels=collabel, loc="right")
        self.table.auto_set_column_width((0, 1, 2))

        # Adjust the main plot to make room for the sliders:
        plt.subplots_adjust(left=0.1, bottom=0.2, right=2.5)
        plt.axis("equal")
        plt.xlim(-0.3, 0.9)
        plt.ylim(-0.3, 0.9)
        plt.tight_layout()
        plt.grid()

        # Make a horizontal slider to control ankle_x:
        ax_ankle_x = plt.axes([0.09, 0.01, 0.58, 0.02])
        self.x_slider = Slider(
            ax=ax_ankle_x,
            label="",
            valmin=0.0,
            valmax=0.6,
            valinit=0.0,
        )

        # Make a vertically oriented slider to control ankle_y:
        ax_ankle_y = plt.axes([0.01, 0.08, 0.02, 0.885])
        self.y_slider = Slider(
            ax=ax_ankle_y,
            label="",
            valmin=-0.3,
            valmax=0.3,
            valinit=0.0,
            orientation="vertical",
        )

        # Make a slider to control hip_fraction:
        ax_horizontal = plt.axes([0.74, 0.85, 0.2, 0.02])
        self.hip_slider = Slider(
            ax=ax_horizontal,
            label="hip",
            valmin=0.0,
            valmax=1.0,
            valinit=0.5,
        )

        # Make a slider to control knee_bed:
        ax_horizontal = plt.axes([0.74, 0.9, 0.2, 0.02])
        self.knee_slider = Slider(
            ax=ax_horizontal,
            label="knee",
            valmin=0,
            valmax=10,
            valinit=8,
        )

        # register the update function with each slider
        self.x_slider.on_changed(self.update)
        self.y_slider.on_changed(self.update)
        self.hip_slider.on_changed(self.update)
        self.knee_slider.on_changed(self.update)

        # create a reset button for all sliders:
        ax_reset = plt.axes([0.7, 0.05, 0.25, 0.04])
        reset_button = Button(ax_reset, "Reset", hovercolor="0.975")
        reset_button.on_clicked(self.reset)

        # create a toggle for stance dorsi flexion:
        ax_toggle = plt.axes([0.7, 0.25, 0.1, 0.04])
        self.toggle_df_rear = Button(
            ax_toggle, "df_rear", color="red", hovercolor="green"
        )
        self.toggle_df_rear.on_clicked(self.toggle_rear)

        # create a toggle for swing dorsi flexion:
        ax_toggle = plt.axes([0.85, 0.25, 0.1, 0.04])
        self.toggle_df_front = Button(
            ax_toggle, "df_front", color="red", hovercolor="green"
        )
        self.toggle_df_front.on_clicked(self.toggle_front)

        # Show all:
        plt.show()

    # The function to be called anytime a slider's value changes
    def update(self, update_value):

        # Get new exo pose:
        pose = Pose()
        pose.solve_all(
            self.x_slider.val,
            self.y_slider.val,
            self.hip_slider.val,
            np.deg2rad(self.knee_slider.val),
            self.reduce_df_front,
            self.reduce_df_rear,
        )
        positions = pose.calculate_joint_positions()
        positions_x = [pos[0] for pos in positions]
        positions_y = [pos[1] for pos in positions]

        self.exo.set_xdata(positions_x)
        self.exo.set_ydata(positions_y)

        # Plot new ankle goal location:
        self.goal.set_xdata(self.x_slider.val)
        self.goal.set_ydata(self.y_slider.val)

        # Plot new toes goal location:
        self.toes.set_xdata(self.x_slider.val + LENGTH_FOOT)
        self.toes.set_ydata(self.y_slider.val)

        pose_rad = pose.pose_left
        for i in np.arange(len(pose_rad)):
            self.table.get_celld()[(i + 1, 1)].get_text().set_text(
                np.round(pose_rad[i], 2)
            )
            self.table.get_celld()[(i + 1, 2)].get_text().set_text(
                np.round(np.rad2deg(pose_rad[i]), 2)
            )

        self.fig.canvas.draw_idle()

    # reset function:
    def reset(self, event):
        self.x_slider.reset()
        self.y_slider.reset()
        self.hip_slider.reset()
        self.knee_slider.reset()

    # toggle function:
    def toggle_rear(self, event):
        if self.reduce_df_rear:
            self.reduce_df_rear = False
            self.toggle_df_rear.color = "red"
            self.toggle_df_rear.hovercolor = "green"
        else:
            self.reduce_df_rear = True
            self.toggle_df_rear.color = "green"
            self.toggle_df_rear.hovercolor = "red"
        self.update(0)

    def toggle_front(self, event):
        if self.reduce_df_front:
            self.reduce_df_front = False
            self.toggle_df_front.color = "red"
            self.toggle_df_front.hovercolor = "green"
        else:
            self.reduce_df_front = True
            self.toggle_df_front.color = "green"
            self.toggle_df_front.hovercolor = "red"
        self.update(0)


widget = LiveWidget()
