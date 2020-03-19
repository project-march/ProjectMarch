import math

from PyQt5.QtCore import pyqtSignal
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore


# Enable antialiasing for prettier plots
pg.setConfigOptions(antialias=True)


class JointPlot(pg.PlotItem):
    # Custom signals

    # time, position, button_pressed
    add_setpoint = pyqtSignal(float, float, QtCore.Qt.KeyboardModifiers)

    # index
    remove_setpoint = pyqtSignal(int)

    def __init__(self, joint, show_velocity_plot=False, show_effort_plot=False):
        pg.PlotItem.__init__(self)

        self.dragPoint = None
        self.dragIndex = -1
        self.dragOffset = 0
        self.plot_item = None
        self.plot_position_interpolation = None
        self.plot_velocity_interpolation = None
        self.plot_min_effort = None
        self.plot_max_effort = None

        self.velocity_markers = []

        # Store the velocities so a plot can be converted to a setpoint.
        self.velocities = []

        self.limits = joint.limits
        self.lower_limit = math.degrees(self.limits.lower)
        self.upper_limit = math.degrees(self.limits.upper)

        self.duration = joint.duration
        self.joint = joint

        self.create_plots(joint)

        self.setTitle(joint.name)

        self.setXRange(-0.1, self.duration + 0.1, padding=0)
        self.setYRange(self.lower_limit - 0.1, self.upper_limit + 0.1, padding=0)
        middle_y = (self.upper_limit + self.lower_limit) / 2
        self.zero_line = self.addLine(y=middle_y)
        limit_pen = pg.mkPen(color='r', style=QtCore.Qt.DotLine)
        self.addLine(y=self.lower_limit, pen=limit_pen)
        self.addLine(y=self.upper_limit, pen=limit_pen)

        time_pen = pg.mkPen(color='y', style=QtCore.Qt.DotLine)
        self.time_line = self.addLine(0, pen=time_pen, bounds=(0, self.duration))

        self.setMouseEnabled(False, False)
        self.setMenuEnabled(False)
        self.hideButtons()

        self.update_set_points(joint, show_velocity_plot, show_effort_plot)

    def create_velocity_markers(self, setpoints, display=False):
        # Remove old sliders
        while self.velocity_markers:
            self.removeItem(self.velocity_markers.pop())
            self.velocities.pop()

        marker_length = self.duration / 10.0
        if display:
            velocity_pen = pg.mkPen(color='g', size=3)
        else:
            velocity_pen = None

        for setpoint in setpoints:

            # Calculate start and endpoint of velocity marker
            dx = 0.5 * marker_length * math.cos(math.atan(setpoint.velocity))
            x_start = setpoint.time - dx
            x_end = setpoint.time + dx

            dy = math.degrees(0.5 * marker_length * math.sin(math.atan(setpoint.velocity)))
            y_start = math.degrees(setpoint.position) - dy
            y_end = math.degrees(setpoint.position) + dy

            self.velocity_markers.append(self.plot([x_start, x_end], [y_start, y_end], pen=velocity_pen))
            self.velocities.append(setpoint.velocity)

    def update_time_slider(self, time):
        self.time_line.setValue(time)

    def create_plots(self, joint):
        self.plot_item = self.plot(pen=None, symbolBrush=(255, 0, 0), symbolPen='w')
        self.showGrid(True, True, 1)
        self.plot_position_interpolation = self.plot()
        self.plot_velocity_interpolation = self.plot(pen=pg.mkPen(color='g'))
        self.plot_min_effort = self.plot(pen=pg.mkPen(color='r'))
        self.plot_max_effort = self.plot(pen=pg.mkPen(color='r'))

    def update_set_points(self, joint, show_velocity_plot=False, show_effort_plot=False):
        time, position, velocity = joint.get_setpoints_unzipped()

        self.duration = joint.duration
        self.setXRange(-0.1, self.duration + 0.1, padding=0)
        self.time_line.setBounds((0, self.duration))

        for i in range(0, len(position)):
            position[i] = math.degrees(position[i])

        self.create_velocity_markers(joint.setpoints, show_velocity_plot)

        self.plot_item.setData(time, position)

        [indices, position_data, velocity_data] = joint.interpolate_setpoints()
        min_effort_data, max_effort_data = self.calculate_min_max_effort(position_data, velocity_data)
        for i in range(0, len(position_data)):
            position_data[i] = math.degrees(position_data[i])
            velocity_data[i] = self.scale_parameter(velocity_data[i], self.limits.velocity)
            min_effort_data[i] = self.scale_parameter(min_effort_data[i], self.limits.effort)
            max_effort_data[i] = self.scale_parameter(max_effort_data[i], self.limits.effort)

        self.plot_position_interpolation.setData(indices, position_data)
        self.zero_line.setPen(None)
        if show_velocity_plot:
            self.plot_velocity_interpolation.setData(indices, velocity_data)
            self.zero_line.setPen(pg.mkPen(color=(0, 0, 200), style=QtCore.Qt.DashLine))
        else:
            self.plot_velocity_interpolation.clear()
        if show_effort_plot:
            self.plot_min_effort.setData(indices, min_effort_data)
            self.plot_max_effort.setData(indices, max_effort_data)
            self.zero_line.setPen(pg.mkPen(color=(0, 0, 200), style=QtCore.Qt.DashLine))
        else:
            self.plot_min_effort.clear()
            self.plot_max_effort.clear()

    def calculate_min_max_effort(self, position, velocity):
        max_velocity = [0] * len(position)
        min_velocity = [0] * len(position)
        max_effort = [0] * len(position)
        min_effort = [0] * len(position)
        for i in range(len(position)):
            max_velocity[i] = min(-self.limits.k_position * (position[i] - self.limits.upper), self.limits.velocity)
            min_velocity[i] = max(-self.limits.k_position * (position[i] - self.limits.lower), -self.limits.velocity)
            max_effort[i] = min(-self.limits.k_velocity * (velocity[i] - max_velocity[i]), self.limits.effort)
            min_effort[i] = max(-self.limits.k_velocity * (velocity[i] - min_velocity[i]), -self.limits.effort)
        return min_effort, max_effort

    def scale_parameter(self, parameter, max_paramter, min_parameter=None):
        if min_parameter is None:
            min_parameter = -max_paramter
        plot_range = self.upper_limit - self.lower_limit
        parameter_range = max_paramter - min_parameter
        return self.lower_limit + plot_range * (parameter - min_parameter) / parameter_range

    def mouseClickEvent(self, event):

        scene_position = event.scenePos()

        local_position = self.vb.mapSceneToView(scene_position)

        # Check for deletion
        if event.modifiers() == QtCore.Qt.ShiftModifier:
            # Only allow deletion when there are more than 2 items left.
            if len(self.plot_item.getData()[0]) <= 2:
                return
            for item in self.dataItems:
                new_pts = item.scatter.pointsAt(local_position)
                if len(new_pts) >= 1:
                    self.dragPoint = new_pts[0]
                    index = item.scatter.points().tolist().index(new_pts[0])
                    self.remove_setpoint.emit(index)
                    event.accept()
            return

        # Create a new point
        time = self.getViewBox().mapSceneToView(event.scenePos()).x()
        position = self.getViewBox().mapSceneToView(event.scenePos()).y()

        # If the time is smaller than 0, only create a setpoint with time 0 if no other exists yet
        x, y = self.plot_item.getData()
        if time < 0:
            if 0 in x:
                return
            time = 0
        # And the same for the duration
        if time > self.duration:
            if self.duration in x:
                return
            time = self.duration

        self.add_setpoint.emit(time, math.radians(position), event.modifiers())
        event.accept()

    def mouseDragEvent(self, ev):
        # Check to make sure the button is the left mouse button. If not, ignore it.
        # Would have to change this if porting to Mac I assume.
        if ev.button() != QtCore.Qt.LeftButton:
            ev.ignore()
            return

        if ev.isStart():
            # We are already one step into the drag.
            # Find the point(s) at the mouse cursor when the button was first
            # pressed:
            # pos = ev.buttonDownPos()
            pos = ev.buttonDownScenePos()
            # Switch position into local coords using viewbox
            local_pos = self.vb.mapSceneToView(pos)
            self.joint.save_setpoints()

            for item in self.dataItems:
                new_pts = item.scatter.pointsAt(local_pos)
                if len(new_pts) >= 1:
                    # Store the drag point and the index of the point for future reference.
                    self.dragPoint = new_pts[0]
                    self.dragIndex = item.scatter.points().tolist().index(new_pts[0])
                    # Find the initial offset of the drag operation from the current point.
                    # This value should allow for the initial mismatch from cursor to point to be accounted for.
                    self.dragOffset = new_pts[0].pos() - local_pos
                    # If we get here, accept the event to prevent the screen from panning during the drag.
                    ev.accept()
        elif ev.isFinish():
            # The drag is finished. Reset the drag point and index.
            self.dragPoint = None
            self.dragIndex = -1
            return
        else:
            # If we get here, this isn't the start or end. Somewhere in the middle.
            if self.dragPoint is None:
                # We aren't dragging a point so ignore the event and allow the panning to continue
                ev.ignore()
                return
            else:
                # We are dragging a point. Find the local position of the event.
                local_pos = self.vb.mapSceneToView(ev.scenePos())

                # Update the point in the PlotDataItem using get/set data.
                # If we had more than one plotdataitem we would need to search/store which item
                # is has a point being moved. For this example we know it is the plot_item object.
                x, y = self.plot_item.getData()
                # Be sure to add in the initial drag offset to each coordinate to account for the initial mismatch.

                # Update the new values, normalized to the position limits and neighbouring setpoints.
                x_min = 0
                x_max = self.duration
                if self.dragIndex > 0:
                    x_min = x[self.dragIndex - 1] + 0.01
                if self.dragIndex < len(x) - 1:
                    x_max = x[self.dragIndex + 1] - 0.01
                x[self.dragIndex] = min(max(local_pos.x() + self.dragOffset.x(), x_min), x_max)
                y[self.dragIndex] = min(max(local_pos.y() + self.dragOffset.y(), self.lower_limit), self.upper_limit)

                # Update the PlotDataItem (this will automatically update the graphics when a change is detected)
                self.plot_item.setData(x, y)
        pass

    pass
