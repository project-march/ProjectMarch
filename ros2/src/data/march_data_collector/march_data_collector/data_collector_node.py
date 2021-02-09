import rclpy
import tf2_ros


def main():
    rclpy.init()
    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer)
    data_collector_node = DataCollector(tf_buffer, feet.keys())

    rclpy.spin(data_collector_node)