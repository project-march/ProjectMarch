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
            '/bezier_visualization',  # Change this to the topic you want to subscribe to
            self.callback,
            10)
        self.visualization_subscriber  # prevent unused variable warnings
        self.figure = plt.figure("Bezier Curve")
        self.ax = self.figure.add_subplot(111, projection='3d')
        self.ax.set_xlim(-0.5, 0.5)
        self.ax.set_ylim(-0.3, 0.3)
        self.ax.set_zlim(0, 0.5)
        self.plots = []

    def callback(self, msg):
        try:
            self.get_logger().info(f"received new message for foot1 with length {len(msg.foot1.poses)}")
            self.get_logger().info(f"received new message for foot2 with length {len(msg.foot2.poses)}")
            # if self.plot: 
            #     self.plot.remove()
            for plot in self.plots:
                plot.remove()
            self.plots.clear()
            if len(msg.foot2.poses) == 0: 
                x = [pose.position.x for pose in msg.foot1.poses]
                z = [pose.position.z for pose in msg.foot1.poses]
                y = [pose.position.y for pose in msg.foot1.poses]
                plot, = self.ax.plot(x, y, z)
                self.plots.append(plot)
                plt.pause(0.001)
            elif len(msg.foot2.poses) > 0: 
                x1 = [pose.position.x for pose in msg.foot1.poses]
                z1 = [pose.position.z for pose in msg.foot1.poses]
                y1 = [pose.position.y for pose in msg.foot1.poses]
                x2 = [pose.position.x for pose in msg.foot2.poses]
                z2 = [pose.position.z for pose in msg.foot2.poses]
                y2 = [pose.position.y for pose in msg.foot2.poses]
                plot1, = self.ax.plot(x1, y1, z1)
                plot2, = self.ax.plot(x2, y2, z2)
                self.plots.extend([plot1,plot2])
                plt.pause(0.001)
        except Exception as e:
            print(e)

def main():
    rclpy.init()
    bezier_visualization = BezierVisualization()
    rclpy.spin(bezier_visualization)
    bezier_visualization.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
