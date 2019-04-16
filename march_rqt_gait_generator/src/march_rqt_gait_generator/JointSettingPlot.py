import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import numpy as np

from march_rqt_gait_generator.model.Setpoint import Setpoint

# Enable antialiasing for prettier plots
pg.setConfigOptions(antialias=True)


class JointSettingPlot(pg.PlotItem):

    def __init__(self, joint, duration):
        pg.PlotItem.__init__(self)

        # Initialize the class instance variables.
        self.dragPoint = None
        self.dragIndex = -1
        self.dragOffset = 0
        self.plot_item = None

        self.joint = joint
        # Create initial plot information for this example.
        self.plotSetpoints(joint.setpoints)

        # Customize dynamic properties.
        # self.setTitle(joint.name)
        self.setYRange(self.joint.limits.lower-0.1, self.joint.limits.upper+0.1, padding=0)
        pen = pg.mkPen(color=(255, 0, 0), style=QtCore.Qt.DotLine)
        self.addItem(pg.InfiniteLine(self.joint.limits.lower, angle=0, pen=pen))
        self.addItem(pg.InfiniteLine(self.joint.limits.upper, angle=0, pen=pen))



        # Customize static properties.
        self.setXRange(-0.1, duration + 0.1, padding=0)
        self.setMouseEnabled(False, False)
        self.hideButtons()

    def plotSetpoints(self, setpoints):
        x = []
        y = []

        for i in range(0, len(setpoints)):
            x.append(setpoints[i].time)
            y.append(setpoints[i].position)
        self.plot_item = None
        self.plot_item = self.plot(x, y, symbolBrush=(255, 0, 0), symbolPen='w')

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
                x[self.dragIndex] = local_pos.x() + self.dragOffset.x()
                y[self.dragIndex] = local_pos.y() + self.dragOffset.y()
                # Update the PlotDataItem (this will automatically update the graphics when a change is detected)
                self.plot_item.setData(x, y)
        pass

    pass
