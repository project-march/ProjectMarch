import rclpy
from march_mpc_visualization.mpc_visualization_listener import MpcListener
from march_mpc_visualization import app
import threading


def main():
    rclpy.init()
    mpc_listener = MpcListener()

    # Add stream method to flask app
    app.add_url_rule('/measurement', 'stream_measurement', mpc_listener.stream_measurement)

    # ROS Node and Flask app need to run in seperatie thread
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(mpc_listener)

    flask_thread = threading.Thread(target=app.run)

    # Daemon, so that it shuts down when main() finishes
    flask_thread.daemon = True
    flask_thread.start()

    try:
        executor.spin()
    except KeyboardInterrupt:
        executor.shutdown()

    rclpy.shutdown()

# def listener_callback(msg):
#     new_data_position = msg.joint[0].estimation.states[0].array[0]
#     mpc_visualization_node.get_logger().info("Listened")
#     print("listened!")

# def ros_callback(msg):
#     print(msg)
# new_data_position = 0



# @app.route("/")
# def stream():
#     response_body = {
#         "x": 5,
#         "y": 10
#     }
#
#     response_body["x"] = new_data_position
#     response = make_response(jsonify(response_body), 200)
#     return response
# rclpy.init()
# # mpc_visualization_node = rclpy.create_node('march_mpc_visualization', context=MpcVisualizer)
# mpc_visualization_node = rclpy.create_node('march_mpc_visualization')
# threading.Thread(target=lambda: mpc_visualization_node).start()
# mpc_visualization_node.create_subscription(MpcMsg, '/march/mpc', listener_callback, 1)
# rclpy.spin(mpc_visualization_node)
# # node.create_subscription(UInt32, '/listener', ros_callback, 1)
# pub = mpc_visualization_node.create_publisher(UInt32, '/talker', 10)

# @app.route('/publish')
# def hello_world():
#     msg = UInt32()
#     msg.data = 1
#     pub.publish(msg)
#
#     return 'Hello, World!'