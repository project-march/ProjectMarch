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

        # self.timer_period = 1.0  # seconds
        # self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.subscription = self.create_subscription(
            PressureSolesData,
            '/march/pressure_sole_data',
            self.listener_callback,
            10)
        
        self.publisher = self.create_publisher(MarkerArray, '/pressure_sole_visualization', 10)        
        
        self.subscription  # prevent unused variable warning

        # the coordinates of the pressure sole pads - left foot
        self.x_coord_left = np.array([0.04,0.04,0.134,0.202,0.2122,0.228,0.265,0.275])
        self.y_coord_left = np.array([0.039,0.013,0.050,0.061,0.025,-0.007,0.038,0.002]) + 0.05

        # the coordinates of the pressure sole pads - right foot
        self.x_coord_right = self.x_coord_left
        self.y_coord_right = self.y_coord_left * -1  - 0.05

        # self.foot_length = 0.3 # m 
        # self.foot_width = 0.1 # m

        # Define the resolution of the heatmap
        # self.resolution = 0.001
        # self.pressure_values = np.zeros(8)

        # Generate the X and Y mesh values for the foot shape
        # x_values = np.arange(-self.foot_width/2, self.foot_width/2, self.resolution)
        # y_values = np.arange(0, self.foot_length, self.resolution)
        # self.X, self.Y = np.meshgrid(x_values, y_values)

        # Create a figure and a 3D plot
        # self.fig = plt.figure()
        # self.ax = self.fig.add_subplot(111, projection='3d')
        # plt.ion()  # Turn on interactive mode for dynamic updates
        # plt.show()

        # Initialize the bar plot
        # self.bars = None # instead of creating a new bar everytime, update the values of the first bar object

    def create_heatmap(self, data):
        # Create a figure and a 3D plot

        # if self.bars:
        #     self.ax.collections.remove(self.bars)

        # x = self.X.ravel()
        # y = self.Y.ravel()
        # top = np.zeros_like(x)
        # bottom = np.zeros_like(top)
        # self.bars = self.ax.bar3d(x, y, bottom, self.resolution, self.resolution, top, shade=True)

        # # left foot

        # z_value = np.array([data[0],data[1],data[2],data[3],data[4],data[5],data[6]*1000,data[7]*1000])

        # self.bars = self.ax.bar3d(x_coord, y_coord, 0, self.resolution, self.resolution, z_value, shade=True)

        # # Customize the plot if needed
        # self.ax.set_xlabel('X')
        # self.ax.set_ylabel('Y')
        # self.ax.set_zlabel('Z')

        # self.fig.canvas.draw()
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
            marker_container.scale.z = data[i]

            marker_container.pose.position.x = self.x_coord_left[i]
            marker_container.pose.position.y = self.y_coord_left[i]
            marker_container.pose.position.z = data[i]/2 # the scale is from the middle of the marker so move the marker up

            marker_container.pose.orientation.z = 0.0
            marker_container.pose.orientation.y = 0.0
            marker_container.pose.orientation.z = 0.0
            marker_container.pose.orientation.w = 1.0
            marker_container.action = 0
            marker_container.frame_locked = True;
            marker_container.color.a = 1.0;
            marker_container.color.b = 0.5;
            marker_container.color.g = 0.7;


            press_vis.markers.append(marker_container)

            marker_container = Marker()
            marker_container.header.frame_id = "map"
            marker_container.type = 1
            marker_container.id = i+8
            marker_container.ns = f'pressure sole vis {i+8}'

            marker_container.scale.x = 0.01
            marker_container.scale.y = 0.01
            marker_container.scale.z = data[i+8]

            marker_container.pose.position.x = self.x_coord_right[i]
            marker_container.pose.position.y = self.y_coord_right[i]
            marker_container.pose.position.z = data[i+8]/2 # the scale is from the middle of the marker so move the marker up

            marker_container.pose.orientation.z = 0.0
            marker_container.pose.orientation.y = 0.0
            marker_container.pose.orientation.z = 0.0
            marker_container.pose.orientation.w = 1.0

            marker_container.action = 0
            marker_container.frame_locked = True;
            marker_container.color.a = 1.0;
            marker_container.color.b = 0.5;
            marker_container.color.g = 0.7;


            press_vis.markers.append(marker_container)


        self.publisher.publish(press_vis)

    # def timer_callback(self):

    def listener_callback(self, msg):
        self.create_heatmap(msg.pressure_values)
        # self.pressure_values = msg.pressure_values

    
    if __name__ == '__main__':
        main()
