import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from march_shared_msgs.msg import PressureSolesData
import numpy as np
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3


NODE_NAME = "pressure_sole_vis"


def main(args=None):
    """Lifecycle of the node."""
    rclpy.init(args=args)
    press_sole_vis = PressureSoleVis()
    rclpy.spin(press_sole_vis)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    press_sole_vis.destroy_node()
    rclpy.shutdown()

    

class PressureSoleVis(Node):
    """This node subscribes to the pressure sole data topic
    it's sole (hah) purpose is to visualize this, there is no publisher for this class
    The order for of the data points, 8 pads per foot
    
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
    - r_arch"""

    def __init__(self):
        super().__init__(NODE_NAME)
        self.subscription = self.create_subscription(
            PressureSolesData,
            '/march/pressure_sole_data',
            self.listener_callback,
            10)
        
        self.publisher = self.create_publisher(MarkerArray, '/pressure_sole_visualization', 10)        
        
        self.subscription  # prevent unused variable warning

        # the coordinates of the pressure sole pads - left foot
        self.x_coord_left = np.array([0.044,0.044,0.138,0.206,0.2162,0.228,0.265,0.275])
        self.y_coord_left = np.array([0.039,0.013,0.050,0.061,0.025,-0.007,0.038,0.002]) + 0.05

        # the coordinates of the pressure sole pads - right foot
        self.x_coord_right = self.x_coord_left
        self.y_coord_right = self.y_coord_left * -1


    def create_heatmap(self, data):
        press_vis = MarkerArray()


        # create markers for all of the pressure sole values
        for i in range(int(len(data)/2)):

            marker_container = Marker()
            marker_container.header.frame_id = "map"
            marker_container.type = 1
            marker_container.id = i
            marker_container.ns = f'pressure sole vis {i}'

            marker_container.scale.x = 0.01
            marker_container.scale.y = 0.01
            marker_container.scale.z = min(data[i],0.1)

            marker_container.pose.position.x = self.x_coord_left[i]
            marker_container.pose.position.y = self.y_coord_left[i]
            marker_container.pose.position.z = marker_container.scale.z/2 # the scale is from the middle of the marker so move the marker up

            marker_container.pose.orientation.z = 0.0
            marker_container.pose.orientation.y = 0.0
            marker_container.pose.orientation.z = 0.0
            marker_container.pose.orientation.w = 1.0
            marker_container.action = 0
            marker_container.frame_locked = True
            marker_container.color.a = 1.0
            marker_container.color.r = 1.0
            marker_container.color.b = 0.0
            marker_container.color.g = 0.0


            press_vis.markers.append(marker_container)

            marker_container = Marker()
            marker_container.header.frame_id = "map"
            marker_container.type = 1
            marker_container.id = i+8
            marker_container.ns = f'pressure sole vis {i+8}'

            marker_container.scale.x = 0.01
            marker_container.scale.y = 0.01
            marker_container.scale.z = min(data[i+8],0.1)

            marker_container.pose.position.x = self.x_coord_right[i]
            marker_container.pose.position.y = self.y_coord_right[i]
            marker_container.pose.position.z = marker_container.scale.z/2 # the scale is from the middle of the marker so move the marker up

            marker_container.pose.orientation.z = 0.0
            marker_container.pose.orientation.y = 0.0
            marker_container.pose.orientation.z = 0.0
            marker_container.pose.orientation.w = 1.0

            marker_container.action = 0
            marker_container.frame_locked = True
            marker_container.color.a = 1.0
            marker_container.color.b = 0.5
            marker_container.color.g = 0.7

            press_vis.markers.append(marker_container)

        marker_left_foot = Marker()
        marker_left_foot.type = 10

        marker_left_foot.header.frame_id = "map"
        marker_left_foot.mesh_resource = "package://march_description/urdf/march8/obj-files/FootLeft.obj"
        marker_left_foot.mesh_use_embedded_materials = True

        marker_left_foot.action = 0
        marker_left_foot.frame_locked = True
        marker_left_foot.scale.x = 1.0
        marker_left_foot.scale.y = 1.0
        marker_left_foot.scale.z = 1.0
        marker_left_foot.ns = "left pressure sole"
        # marker_left_foot.lifetime.sec = 1;

        marker_left_foot.pose.position.x = 0.09
        marker_left_foot.pose.position.y = 0.16
        marker_left_foot.pose.position.z = 0.138

        marker_left_foot.pose.orientation.z = 0.0
        marker_left_foot.pose.orientation.y = 0.0
        marker_left_foot.pose.orientation.z = 0.0
        marker_left_foot.pose.orientation.w = 1.0

        # marker_left_foot.color.a = 1.0
        # marker_left_foot.color.b = 0.5
        # marker_left_foot.color.g = 0.7

        press_vis.markers.append(marker_left_foot)

        marker_right_foot = Marker()

        marker_right_foot.type = 10

        marker_right_foot.header.frame_id = "map"
        marker_right_foot.mesh_resource = "package://march_description/urdf/march8/obj-files/FootRight.obj"
        marker_right_foot.mesh_use_embedded_materials = True

        marker_right_foot.action = 0;
        marker_right_foot.frame_locked = True
        marker_right_foot.scale.x = 1.0
        marker_right_foot.scale.y = 1.0
        marker_right_foot.scale.z = 1.0
        marker_right_foot.ns = "right pressure sole"
        # marker_right_foot.lifetime.sec = 1

        marker_right_foot.pose.position.x = 0.09
        marker_right_foot.pose.position.y = -0.16
        marker_right_foot.pose.position.z = 0.138

        marker_right_foot.pose.orientation.z = 0.0
        marker_right_foot.pose.orientation.y = 0.0
        marker_right_foot.pose.orientation.z = 0.0
        marker_right_foot.pose.orientation.w = 1.0

        press_vis.markers.append(marker_right_foot)

        self.publisher.publish(press_vis)


    def listener_callback(self, msg):
        self.create_heatmap(msg.pressure_values)

    
    if __name__ == '__main__':
        main()
