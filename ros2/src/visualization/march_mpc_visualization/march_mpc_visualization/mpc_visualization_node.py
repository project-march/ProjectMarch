import rclpy
#import rospy
# from march_mpc_visualization.mpc_visualization_listener import MpcListener
from march_shared_msgs.msg import MpcMsg
# from march_mpc_visualization import app
import threading
import os
from flask import Flask
from flask import jsonify, make_response
# from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import UInt32


# def ros_callback(msg):
#     print(msg)
new_data_position = 0

def listener_callback(msg):
    new_data_position = msg.joint[0].estimation.states[0].array[0]
    mpc_visualization_node.get_logger().info('I heard: "%f"' % new_data_position)

rclpy.init()
mpc_visualization_node = rclpy.create_node('march_mpc_visualization')
threading.Thread(target=lambda: mpc_visualization_node).start()
mpc_visualization_node.create_subscription(MpcMsg, '/march/mpc', listener_callback, 1)
# node.create_subscription(UInt32, '/listener', ros_callback, 1)
pub = mpc_visualization_node.create_publisher(UInt32, '/talker', 10)

app = Flask('march_mpc_visualizer')
app = Flask(__name__.split('.')[0])


@app.route("/")
def stream():
    response_body = {
        "x": 5,
        "y": 10
    }

    response_body["x"] = new_data_position
    response = make_response(jsonify(response_body), 200)
    return response

@app.route('/publish')
def hello_world():
    msg = UInt32()
    msg.data = 1
    pub.publish(msg)

    return 'Hello, World!'

def main():
    app.run()

# @app.route('/')
# def stream():
#     # response_body = {
#     #     "x": 5,
#     #     "y": 10
#     # }
#     #
#     # response_body["x"] = new_data_position
#     # response = make_response(jsonify(response_body), 200)
#     return "Hello"

# def main(args=None):
#     """The main function used to start up the rqt note taker."""
#     rclpy.init(args=args)
#
#     mpc_listener = MpcListener()
#     app.add_url_rule('/', 'stream', mpc_listener.stream)
#     #app.run()
#     # executor = MultiThreadedExecutor()
#     #threading.Thread(target=lambda: rclpy.create_node('mpc_listener', use_global_arguments=False)).start()
#     executor = MultiThreadedExecutor()
#     rclpy.spin(mpc_listener, executor)
#     threading.Thread(target=app.run).start()
#     #rclpy.spin(mpc_listener)
#     # try:
#     #     rclpy.spin(mpc_listener)
#     # except KeyboardInterrupt:
#     #     pass
#
#     # rclpy.shutdown()


# if __name__ == '__main__':
#     app.run(host=os.environ['ROS_IP'], port=3000)