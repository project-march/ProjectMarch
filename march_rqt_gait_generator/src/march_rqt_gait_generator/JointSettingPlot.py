import math

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
from PyQt5.QtCore import QObject, pyqtSignal

import rospy

# Enable antialiasing for prettier plots
pg.setConfigOptions(antialias=True)


class JointSettingPlot(pg.PlotItem):
    # Custom signals

    # time, position, button_pressed
    add_setpoint = pyqtSignal(float, float, QtCore.Qt.KeyboardModifiers)

    # index
    remove_setpoint = pyqtSignal(int)

    def __init__(self, joint, duration):
        pg.PlotItem.__init__(self)

        self.dragPoint = None
        self.dragIndex = -1
        self.dragOffset = 0
        self.plot_item = None
        self.plot_interpolation = None

        self.velocity_markers = []

        # Store the velocities so a plot can be converted to a setpoint.
        self.velocities = []

        self.lower_limit = math.degrees(joint.limits.lower)
        self.upper_limit = math.degrees(joint.limits.upper)
        self.duration = duration

        self.createPlots(joint)

        self.setTitle(joint.name)

        self.setYRange(self.lower_limit-0.1, self.upper_limit+0.1, padding=0)
        limit_pen = pg.mkPen(color='r', style=QtCore.Qt.DotLine)
        self.addItem(pg.InfiniteLine(self.lower_limit, angle=0, pen=limit_pen))
        self.addItem(pg.InfiniteLine(self.upper_limit, angle=0, pen=limit_pen))
        self.setXRange(-0.1, self.duration + 0.1, padding=0)
        self.setMouseEnabled(False, False)
        self.setMenuEnabled(False)
        self.hideButtons()

        self.updateSetpoints(joint)

        time_pen = pg.mkPen(color='y', style=QtCore.Qt.DotLine)
        self.time_line = self.addLine(0, pen=time_pen, bounds=(0, self.duration))

    def createVelocityMarkers(self, setpoints):
        # Remove old sliders
        while self.velocity_markers:
            self.removeItem(self.velocity_markers.pop())
            self.velocities.pop()

        marker_length = self.duration/10.0

        for setpoint in setpoints:
            velocity_pen = pg.mkPen(color='g', size=3)

            # Calculate start and endpoint of velocity marker
            dx = 0.5*marker_length*math.cos(math.atan(setpoint.velocity))
            x_start = setpoint.time - dx
            x_end = setpoint.time + dx

            dy = math.degrees(0.5*marker_length*math.sin(math.atan(setpoint.velocity)))
            y_start = math.degrees(setpoint.position) - dy
            y_end = math.degrees(setpoint.position) + dy

            self.velocity_markers.append(self.plot([x_start, x_end], [y_start, y_end], pen=velocity_pen))
            self.velocities.append(setpoint.velocity)

    def updateTimeSlider(self, time):
        self.time_line.setValue(time)

    def createPlots(self, joint):
        self.plot_item = self.plot(pen=None, symbolBrush=(255, 0, 0), symbolPen='w')
        self.showGrid(True, True, 1)
        self.plot_interpolation = self.plot()

    def updateSetpoints(self, joint):
        time, position, velocity = joint.get_setpoints_unzipped()

        for i in range(0, len(position)):
            position[i] = math.degrees(position[i])

        self.createVelocityMarkers(joint.setpoints)

        self.plot_item.setData(time, position)

        [indices, values] = joint.interpolate_setpoints()
        for i in range(0, len(values)):
            values[i] = math.degrees(values[i])

        self.plot_interpolation.setData(indices, values)

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
                    x_min = x[self.dragIndex-1] + 0.01
                if self.dragIndex < len(x)-1:
                    x_max = x[self.dragIndex+1] - 0.01
                x[self.dragIndex] = min(max(local_pos.x() + self.dragOffset.x(), x_min), x_max)
                y[self.dragIndex] = min(max(local_pos.y() + self.dragOffset.y(), self.lower_limit), self.upper_limit)

                # Update the PlotDataItem (this will automatically update the graphics when a change is detected)
                self.plot_item.setData(x, y)
        pass

    pass
