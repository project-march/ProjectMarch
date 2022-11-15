import numpy as np
from mujoco_interfaces.srv import ReadMujoco
from mujoco_interfaces.msg import MujocoDataRequest

from mujoco_interfaces.msg import MujocoDataState
from mujoco_interfaces.msg import MujocoDataSensing
from mujoco_interfaces.msg import MujocoDataControl

import rclpy
from rclpy.node import Node

class Mujoco_readerNode(Node):

    def __init__(self):
        """This node is responsible for obtaining data from Mujoco.
        NOTE: Right now, it only obtains Pose() type messages, but we
        can extend this to be more universal/modular in what data we want
        to obtain.
        """
        super().__init__("mujoco_reader")

        self.declare_parameter('reader_data_type')
        self.request_type = self.get_parameter('reader_data_type').value
        #Create client, which waits for the service to come online
        self.client_read = self.create_client(ReadMujoco,'read_mujoco')
        while not self.client_read.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = ReadMujoco.Request()
        #Timer callback, as we want the reader to periodaically request
        #Mujoco data
        TIMER_CALLBACK = 1
        self.create_timer(TIMER_CALLBACK, self.request_sim_timer_callback)

    def send_request(self):
        """Requests the data from the Mujoco service.
        """
        request_tosend = MujocoDataRequest()
        request_tosend.stamp = self.get_clock().now().to_msg()
        request_tosend.request = self.request_type
        self.req.mujoco_info_type = request_tosend
        self.future = self.client_read.call_async(self.req)
    
    def request_sim_timer_callback(self):
        """Callback timer function to actually perform the request
        """
        self.send_request()


def main(args=None):
    rclpy.init(args=args)
    node = Mujoco_readerNode()
    #The node is structured this way to ensure asynchronous sensor
    #requests. Otherwise everything stalls until the request is answered.
    while rclpy.ok():
        rclpy.spin_once(node)
        if node.future.done():
            try:
                #NOTE: LATER, WE SHOULD ADD SOME PUBLISHING HERE
                response = node.future.result()
            except Exception as e:
                node.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()