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
        self.visualization_subscriber  # prevent unused variable warning

    def callback(self, msg):
        try:
            self.figure = plt.figure("Bezier Curve")
            x = [pose.position.x for pose in msg.poses]
            z = [pose.position.z for pose in msg.poses]
            plt.plot(x, z, color="green")
            self.get_logger().info("Tried to plot bezier points")
            plt.show()
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
