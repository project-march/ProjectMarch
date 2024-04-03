import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt 
from geometry_msgs.msg import PoseArray

class BezierVisualization(Node):
    def __init__(self):
        super().__init__('bezier_visualization_node')
        self.visualization_subscriber = self.create_subscription(
            PoseArray,
            '/bezier_visualization',  # Change this to the topic you want to subscribe to
            self.callback,
            10)
        self.visualization_subscriber  # prevent unused variable warnings
        self.figure = plt.figure("Bezier Curve")
        self.ax = plt.subplot()
        self.plot = None

    def callback(self, msg):
        try:
            self.get_logger().debug(f"received new message with length {len(msg.poses)}")
            if self.plot: 
                self.plot.remove()
            x = [pose.position.x for pose in msg.poses]
            z = [pose.position.z for pose in msg.poses]
            self.plot, = self.ax.plot(x, z)
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
