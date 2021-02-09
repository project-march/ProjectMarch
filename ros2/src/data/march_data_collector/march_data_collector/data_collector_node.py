import rclpy
from march_data_collector.data_collector import DataCollector


def main():
    rclpy.init()

    data_collector_node = DataCollector("data_collector")

    rclpy.spin(data_collector_node)
