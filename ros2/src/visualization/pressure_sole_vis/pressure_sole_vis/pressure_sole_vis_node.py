"""Pressure node visualization tool written by Jack Zeng M8."""

import rclpy
from rclpy.node import Node
from march_shared_msgs.msg import PressureSolesData
import numpy as np
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
import colorsys


NODE_NAME = "pressure_sole_vis"


def main(args=None):
    """Lifecycle of the node."""
    rclpy.init(args=args)
    press_sole_vis = PressureSoleVis()
    rclpy.spin(press_sole_vis)
    press_sole_vis.destroy_node()
    rclpy.shutdown()


class PressureSoleVis(Node):
    """This node subscribes to the pressure sole data topic, it's sole (hah) purpose is to visualize this, there is no publisher for this class.

    The order for of the data points, 8 pads per foot.
    - l_heel_right
    - l_heel_left
    - l_met1
    - l_hallux
    - l_met3
    - l_toes
    - l_met5
    - l_arch
    - r_heel_right
    - r_heel_left
    - r_met1
    - r_hallux
    - r_met3
    - r_toes
    - r_met5
    - r_arch
    """

    def __init__(self):
        super().__init__(NODE_NAME)
        self.subscription = self.create_subscription(
            PressureSolesData,
            '/march/pressure_sole_data',
            self.listener_callback,
            10)

        self.publisher = self.create_publisher(MarkerArray, '/pressure_sole_visualization', 10)
        self.subscription  # prevent unused variable warning
        self.min_z = 0.0
        self.max_z = 1.6
        # the coordinates of the pressure sole pads - left foot
        self.x_coord_left = np.array([0.044, 0.044, 0.228, 0.275, 0.219, 0.265, 0.206, 0.139])
        self.y_coord_left = np.array([0.039, 0.012, -0.007, 0.005, 0.026, 0.038, 0.061, 0.049]) + 0.05
        # the coordinates of the pressure sole pads - right foot
        self.x_coord_right = self.x_coord_left
        self.y_coord_right = self.y_coord_left * -1
        self.previous_data = np.zeros(16)  # amount of pressure pads

    def hsv_to_rgb(self, h, s, v):
        """Convert hsv to rgb colour pattern, where h is the magnitude of the pressure sole data."""
        if h is None:
            h = 0
        r, g, b = colorsys.hsv_to_rgb(0.667 - (h * 0.667), s, v)
        return r, g, b

    def create_heatmap(self, data):
        """Create a 3D barplot heatmap of the pressure sole data."""
        press_vis = MarkerArray()
        data = 3.5 - np.array(data)
        effective_data = data * 0.5 + self.previous_data * 0.5
        self.previous_data = data
        resolution = 150  # amount of markers that are created

        # create markers for all of the pressure sole values
        for i in range(int(len(data) / 2)):
            marker_container_strip = Marker()
            marker_container_strip.type = 6
            marker_container_strip.header.frame_id = "map"

            marker_container_strip.scale.x = 0.033
            marker_container_strip.scale.y = 0.02
            marker_container_strip.scale.z = 0.1 / resolution

            marker_container_strip.pose.position.y = self.y_coord_left[i]
            marker_container_strip.pose.position.x = self.x_coord_left[i]
            marker_container_strip.pose.position.z = 0.0

            marker_container_strip.pose.orientation.x = 0.0
            marker_container_strip.pose.orientation.y = 0.0
            marker_container_strip.pose.orientation.z = 0.0

            if i in [2, 7]:
                marker_container_strip.pose.orientation.z = 0.08
            elif i in [3, 5]:
                marker_container_strip.pose.orientation.z = -0.01
            elif i == 6:
                marker_container_strip.pose.orientation.z = -0.4

            marker_container_strip.pose.orientation.w = 1.0
            marker_container_strip.id = i
            marker_container_strip.ns = f'pressure sole vis {(i + 1)}'

            marker_container_strip.action = 0
            marker_container_strip.frame_locked = True
            press_vis.markers.append(marker_container_strip)

            for j in range(int(effective_data[i] * resolution)):  # something that creates an amount of markers based on the height
                point_container = Point()
                color = ColorRGBA()
                color.a = 1.0
                normalized_z = (effective_data[i] - self.min_z) / (self.max_z - self.min_z) if self.max_z != self.min_z else 0.2

                point_container.x = 0.0
                point_container.y = 0.0
                point_container.z = j * marker_container_strip.scale.z  # the scale is from the middle of the point so move the point up

                color.r, color.g, color.b = self.hsv_to_rgb((normalized_z / (int(effective_data[i] * resolution)) * j), 1.0, 1.0)

                marker_container_strip.points.append(point_container)
                marker_container_strip.colors.append(color)

            marker_container_strip = Marker()
            marker_container_strip.type = 6
            marker_container_strip.header.frame_id = "map"

            marker_container_strip.scale.x = 0.033
            marker_container_strip.scale.y = 0.02
            marker_container_strip.scale.z = 0.1 / resolution

            marker_container_strip.pose.position.y = self.y_coord_right[i]
            marker_container_strip.pose.position.x = self.x_coord_right[i]
            marker_container_strip.pose.position.z = 0.0

            marker_container_strip.pose.orientation.x = 0.0
            marker_container_strip.pose.orientation.y = 0.0
            marker_container_strip.pose.orientation.z = 0.0

            if i in [2, 7]:
                marker_container_strip.pose.orientation.z = -0.08
            elif i in [3, 5]:
                marker_container_strip.pose.orientation.z = 0.01
            elif i == 6:
                marker_container_strip.pose.orientation.z = 0.4

            marker_container_strip.pose.orientation.w = 1.0
            marker_container_strip.id = (i + 8)
            marker_container_strip.ns = f'pressure sole vis {(i + 9)}'

            marker_container_strip.action = 0
            marker_container_strip.frame_locked = True
            press_vis.markers.append(marker_container_strip)

            for j in range(int(effective_data[i + 8] * resolution)):  # something that creates an amount of markers based on the height
                point_container = Point()
                color = ColorRGBA()
                color.a = 1.0
                normalized_z = (effective_data[i + 8] - self.min_z) / (self.max_z - self.min_z) if self.max_z != self.min_z else 0.2

                point_container.x = 0.0
                point_container.y = 0.0
                point_container.z = j * marker_container_strip.scale.z  # the scale is from the middle of the point so move the point up

                color.r, color.g, color.b = self.hsv_to_rgb((normalized_z / (int(effective_data[i + 8] * resolution)) * j), 1.0, 1.0)

                marker_container_strip.points.append(point_container)
                marker_container_strip.colors.append(color)

        marker_left_foot = Marker()
        marker_left_foot.type = 10

        marker_left_foot.header.frame_id = "map"
        marker_left_foot.mesh_resource = "package://march_description/urdf/march8/obj-files/FootLeft.obj"
        marker_left_foot.mesh_use_embedded_materials = True

        marker_left_foot.ns = "left pressure sole"
        marker_left_foot.id = 1
        marker_left_foot.action = 0
        marker_left_foot.frame_locked = True
        marker_left_foot.scale.x = 1.0
        marker_left_foot.scale.y = 1.0
        marker_left_foot.scale.z = 1.0

        marker_left_foot.pose.position.x = 0.09
        marker_left_foot.pose.position.y = 0.16
        marker_left_foot.pose.position.z = 0.138

        marker_left_foot.pose.orientation.z = 0.0
        marker_left_foot.pose.orientation.y = 0.0
        marker_left_foot.pose.orientation.z = 0.0
        marker_left_foot.pose.orientation.w = 1.0

        marker_right_foot = Marker()

        marker_right_foot.type = 10
        marker_right_foot.header.frame_id = "map"
        marker_right_foot.mesh_resource = "package://march_description/urdf/march8/obj-files/FootRight.obj"
        marker_right_foot.mesh_use_embedded_materials = True

        marker_right_foot.ns = "right pressure sole"
        marker_right_foot.id = 1
        marker_right_foot.action = 0
        marker_right_foot.frame_locked = True
        marker_right_foot.scale.x = 1.0
        marker_right_foot.scale.y = 1.0
        marker_right_foot.scale.z = 1.0

        marker_right_foot.pose.position.x = 0.09
        marker_right_foot.pose.position.y = -0.16
        marker_right_foot.pose.position.z = 0.138

        marker_right_foot.pose.orientation.z = 0.0
        marker_right_foot.pose.orientation.y = 0.0
        marker_right_foot.pose.orientation.z = 0.0
        marker_right_foot.pose.orientation.w = 1.0

        press_vis.markers.append(marker_right_foot)
        press_vis.markers.append(marker_left_foot)

        self.publisher.publish(press_vis)

    def listener_callback(self, msg):
        """Callback for subscriber to the pressure sole data topic."""
        self.create_heatmap(msg.pressure_values)

    if __name__ == '__main__':
        main()
