import rclpy
from rclpy.node import Node
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseArray
from march_shared_msgs.msg import VisualizationBeziers
from mpl_toolkits.mplot3d import Axes3D

class BezierVisualization(Node):
    def __init__(self):
        super().__init__('bezier_visualization_node')
        self.visualization_subscriber = self.create_subscription(
            VisualizationBeziers,
            '/bezier_visualization', 
            self.callback,
            10)
        self.figure = plt.figure("Bezier Curve")
        self.ax = self.figure.add_subplot(111, projection='3d')
        self.ax.set_xlim(-0.5, 0.5)
        self.ax.set_ylim(-0.3, 0.3)
        self.ax.set_zlim(0, 0.5)
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.plots = []
        self.data = {'foot1': [], 'foot2': []} 

        # Create a timer to update the plot every 100 milliseconds
        self.timer = self.create_timer(0.1, self.update_plot)

        # Show the plot
        plt.ion()  # Interactive mode on
        plt.show()

    def callback(self, msg):
        try:
            self.get_logger().info(f"received new message for foot1 with length {len(msg.foot1.poses)}")
            self.get_logger().info(f"received new message for foot2 with length {len(msg.foot2.poses)}")

            # Store the new data
            self.data['foot1'] = [(pose.position.x, pose.position.y, pose.position.z) for pose in msg.foot1.poses]
            self.data['foot2'] = [(pose.position.x, pose.position.y, pose.position.z) for pose in msg.foot2.poses]

        except Exception as e:
            print(e)

    def update_plot(self):
        # Remove previous plots
        while self.plots:
            plot = self.plots.pop()
            plot.remove()

        # Plot new data
        if self.data['foot1']:
            x1, y1, z1 = zip(*self.data['foot1'])
            plot1, = self.ax.plot(x1, y1, z1, 'b-')
            self.plots.append(plot1)

        if self.data['foot2']:
            x2, y2, z2 = zip(*self.data['foot2'])
            plot2, = self.ax.plot(x2, y2, z2, 'r-')
            self.plots.append(plot2)

        self.figure.canvas.draw_idle()
        plt.pause(0.001)

def main():
    rclpy.init()
    bezier_visualization = BezierVisualization()
    rclpy.spin(bezier_visualization)
    bezier_visualization.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()