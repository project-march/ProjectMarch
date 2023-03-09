import sys
import rclpy

import math
from matplotlib.path import Path
import matplotlib.patches as patches
import matplotlib.pyplot as plt
from matplotlib.backend_bases import MouseEvent

NODE_NAME = "bezier_plotter"


def sys_exit(*_):
    """Cleanly exit."""
    sys.exit(0)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

class BezierCurve(Node):
    def __init__(self, points):
        super().__init__('NODE_NAME')
        self.subscription = self.create_subscription(
            String,
            'bezier_points',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.dragging_point, self.line, self.codes, self.path, self.patch, self.legend_handles, self.labels = None, None, None, None, None, None, None
        self.figure = plt.figure("Bezier Curve")
        self.points = points
        self.axes = plt.subplot(1, 1, 1)
        self._init_plot()

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
    def _init_plot(self):
        # Set the initial figure with the axes
        self.axes.set_xlim(0, 100)
        self.axes.set_ylim(0, 100)
        self.axes.grid(which="both")

        # Draw the initial line
        x, y = zip(*sorted(self.points.items()))
        self.line, = self.axes.plot(x, y, "b", marker="o", markersize=10)

        # Draw the initial Bézier curve
        self.codes = [Path.MOVETO, Path.CURVE4, Path.CURVE4, Path.CURVE4]
        points_tuple = [(x, y) for x, y in self.points.items()]
        self.path = Path(points_tuple, self.codes)
        self.patch = patches.PathPatch(self.path, facecolor='none', lw=2)
        self.axes.add_patch(self.patch)

        # Connect the events
        self.figure.canvas.mpl_connect('button_press_event', self._on_click)
        self.figure.canvas.mpl_connect('motion_notify_event', self._on_motion)
        self.figure.canvas.mpl_connect('button_release_event', self._on_release)

        # Show the plot
        plt.show()

    def _on_click(self, event):
        """ callback method for mouse click event
        :type event: MouseEvent
        """
        # Only respond to left click within the axes, right click is not relevant
        if event.button == 1 and event.inaxes in [self.axes]:
            # Check if the click is close to a point
            distance_threshold = 2.0
            nearest_point = None
            min_distance = math.sqrt(2 * (100 ** 2))
            for x, y in self.points.items():
                distance = math.hypot(event.xdata - x, event.ydata - y)
                if distance < min_distance:
                    min_distance = distance
                    nearest_point = (x, y)
            # If the click is close to a point, start dragging it
            if min_distance < distance_threshold:
                self.dragging_point = nearest_point

    def _on_motion(self, event):
        """ callback method for mouse motion event
        :type event: MouseEvent
        """
        if not self.dragging_point:
            return
        if event.xdata is None or event.ydata is None:
            return

        # Redraw the points and the line by removing the old ones and adding the new ones
        # First remove the points and the Bezier curve
        try:
            self.points.pop(self.dragging_point[0]) if self.dragging_point[0] in self.points else None
            self.axes.patches.remove(self.patch)
        except ValueError:
            raise ValueError(f'Don\'t drag the points to close to each other. Restart the visualisation.')

        # Recalculate the points
        if isinstance(event, MouseEvent):
            x, y = int(event.xdata), int(event.ydata)
        self.points[x] = y
        self.dragging_point = x, y
        x, y = zip(*sorted(self.points.items()))

        # Recalculate the Bezier curve
        points_tuple = [(x, y) for x, y in sorted(self.points.items())]
        self.path = Path(points_tuple, self.codes)
        self.patch = patches.PathPatch(self.path, facecolor='none', lw=2)

        # Now redraw the points and the Bezier curve on the plot
        self.line.set_data(x, y)
        self.axes.add_patch(self.patch)
        self.figure.canvas.draw()

    def _on_release(self, _):
        """
            callback method for mouse release event. Set the dragging point to None to stop the drag
            :param _: MouseEvent. Mandatory parameter for the callback method
        """
        self.dragging_point = None


plot = BezierCurve({1: 0, 25: 50, 75: 75, 99: 0})
