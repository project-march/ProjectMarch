import math

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
from PyQt5.QtCore import QObject, pyqtSignal

import rospy

# Enable antialiasing for prettier plots
pg.setConfigOptions(antialias=True)


class JointSettingPlot(pg.PlotItem):

    VELOCITY_SLIDER_LENGTH = 1

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
        self.velocity_sliders = []

        self.lower_limit = joint.limits.lower
        self.upper_limit = joint.limits.upper
        self.duration = duration

        self.createPlots(joint)

        self.setTitle(joint.name)

        self.setYRange(self.lower_limit-0.1, self.upper_limit+0.1, padding=0)
        limit_pen = pg.mkPen(color='r', style=QtCore.Qt.DotLine)
        self.addItem(pg.InfiniteLine(joint.limits.lower, angle=0, pen=limit_pen))
        self.addItem(pg.InfiniteLine(joint.limits.upper, angle=0, pen=limit_pen))
        self.setXRange(-0.1, self.duration + 0.1, padding=0)
        self.setMouseEnabled(False, False)
        self.setMenuEnabled(False)
        self.hideButtons()

        self.updateSetpoints(joint)

        time_pen = pg.mkPen(color='y', style=QtCore.Qt.DotLine)
        self.time_line = self.addLine(0, pen=time_pen, bounds=(0, self.duration))

    def createVelocitySliders(self, setpoints):
        # Remove old sliders
        while self.velocity_sliders:
            self.removeItem(self.velocity_sliders.pop())

        for setpoint in setpoints:
            angle = math.degrees(math.atan(setpoint.velocity))
            rospy.logwarn("Angle in deg: " + str(angle))
            velocity_pen = pg.mkPen(color='b', size=3)
            x, y = self.calculate_center_position(setpoint.time, setpoint.position, setpoint.velocity, self.VELOCITY_SLIDER_LENGTH)
            velocity_slider = pg.ROI(pos=(x, y), size=(self.VELOCITY_SLIDER_LENGTH, 0), angle=angle, pen=velocity_pen)
            velocity_slider.addRotateHandle((0, 0), (0.5, 0))
            rospy.logwarn("Angle of slider: " + str(velocity_slider.angle()))
            self.addItem(velocity_slider)
            self.velocity_sliders.append(velocity_slider)

    def updateTimeSlider(self, time):
        self.time_line.setValue(time)

    def createPlots(self, joint):
        self.plot_item = self.plot(pen=None, symbolBrush=(255, 0, 0), symbolPen='w')
        self.showGrid(True, True, 1)
        self.plot_interpolation = self.plot()

    def updateSetpoints(self, joint):
        time, position, velocity = joint.get_setpoints_unzipped()

        self.createVelocitySliders(joint.setpoints)

        self.plot_item.setData(time, position)

        [indices, values] = joint.interpolate_setpoints()
        self.plot_interpolation.setData(indices, values)

        for i in range(0, len(joint.setpoints)):
            angle = velocity[i]*180/math.pi
            self.velocity_sliders[i].setAngle(angle)
            x, y = self.calculate_center_position(time[i], position[i], velocity[i], self.VELOCITY_SLIDER_LENGTH)
            self.velocity_sliders[i].setPos(x, y)

    def mouseClickEvent(self, ev):

        scene_position = ev.scenePos()

        local_position = self.vb.mapSceneToView(scene_position)

        # Check for deletion
        if ev.modifiers() == QtCore.Qt.ShiftModifier:
            for item in self.dataItems:
                new_pts = item.scatter.pointsAt(local_position)
                if len(new_pts) >= 1:
                    self.dragPoint = new_pts[0]
                    index = item.scatter.points().tolist().index(new_pts[0])
                    self.remove_setpoint.emit(index)
                    ev.accept()
            return

        # Create a new point
        time = self.getViewBox().mapSceneToView(ev.scenePos()).x()
        position = self.getViewBox().mapSceneToView(ev.scenePos()).y()

        self.add_setpoint.emit(time, position, ev.modifiers())

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

    def calculate_center_position(self, x, y, angle, length):
        x_center = x - (0.5*length*math.cos(angle))
        y_center = y - (0.5*length*math.sin(angle))
        return x_center, y_center
