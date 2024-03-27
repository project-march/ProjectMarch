import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import matplotlib.pyplot as plt 
from std_msgs.msg import Float64MultiArray

class BezierVisualization(Node):
    def __init__(self):
        super().__init__('bezier_visualization_node')
        self.visualization_subscriber = self.create_subscription(
            Float64MultiArray,
            '/bezier_visualization',  # Change this to the topic you want to subscribe to
            self.callback,
            10)
        self.visualization_subscriber  # prevent unused variable warning
        self.figure = plt.figure("Bezier Curve")

    def callback(self, msg):
        try:
            print(msg.size())
        except Exception as e:
            print(e)

def main():
    rclpy.init()
    bezier_visualization = BezierVisualization()
    rclpy.spin(bezier_visualization)
    bezier_visualization.destroy_node()
    rclpy.shutdown()