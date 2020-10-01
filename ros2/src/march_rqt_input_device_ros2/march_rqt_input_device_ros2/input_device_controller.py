import getpass
import socket
from time import sleep
import rclpy
from rclpy.time import Time as RclTime
from rclpy.clock import ClockType
from rclpy.node import Node
from rclpy.clock import ROSClock
from rclpy.parameter import Parameter
from std_msgs.msg import Header, String
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time
from march_shared_msgs.msg import Alive, Error, GaitInstruction, GaitInstructionResponse
from march_shared_msgs.srv import PossibleGaits

class InputDeviceController(object):

    # Format of the identifier for the alive message
    ID_FORMAT = 'rqt@{machine}@{user}ros2'

    def __init__(self, node, ping):
        # super().__init__()
        self._node = node
        self._instruction_gait_pub = self._node.create_publisher(GaitInstruction, '/march/input_device/instruction', 10)
        self._instruction_response_pub = self._node.create_subscription(GaitInstructionResponse,
                                                                        '/march/input_device/instruction_response',
                                                                        self._response_callback, 10)
        self._current_gait = self._node.create_subscription(String, '/march/gait/current',
                                                            self._current_gait_callback, 1)
        self._error_pub = self._node.create_publisher(Error, '/march/error', 10)
        self._possible_gait_client = self._node.create_client(PossibleGaits, '/march/gait_selection/get_possible_gaits')

        self.accepted_cb = None
        self.finished_cb = None
        self.rejected_cb = None
        self.current_gait_cb = None
        self._node.get_logger().info(str(type(self._node.get_clock())))
        self._node.get_logger().info(str(self._node.get_clock()))
        self._node.get_logger().info(str(self._node.get_clock().now().nanoseconds))
        self._ping = ping
        self._id = self.ID_FORMAT.format(machine=socket.gethostname(),
                                         user=getpass.getuser())
        self._node.get_logger().info(str(self._ping))
        self._timesource = self._node.create_subscription(Clock, '/clock', self._clock_callback, 10)

        if self._ping:
            self._alive_pub = self._node.create_publisher(Alive, '/march/input_device/alive', 10)
            period = 0.2
            self._alive_timer = self._node.create_timer(period, self._timer_callback, clock=self._node.get_clock())

        self.gait_future = self._possible_gait_client.call_async(PossibleGaits.Request())

    def __del__(self):
        self._node.destroy_publisher(self._instruction_gait_pub)
        self._node.destroy_publisher(self._error_pub)
        if self._ping:
            self._alive_timer.shutdown()
            self._alive_timer.join()
            self._alive_pub.unregister()

    def _response_callback(self, msg):
        """
        Callback for instruction response messages.

        Calls registered callbacks when the gait is accepted, finished or rejected.

        :type msg: GaitInstructionResponse
        """
        if msg.result == GaitInstructionResponse.GAIT_ACCEPTED:
            if callable(self.accepted_cb):
                self.accepted_cb()
        elif msg.result == GaitInstructionResponse.GAIT_FINISHED:
            if callable(self.finished_cb):
                self.finished_cb()
        elif msg.result == GaitInstructionResponse.GAIT_REJECTED:
            if callable(self.rejected_cb):
                self.rejected_cb()

    def _current_gait_callback(self, msg):
        if callable(self.current_gait_cb):
            self.current_gait_cb(msg.data)

    def _timer_callback(self):
        msg = Alive(stamp=self._node.get_clock().now().to_msg(), id=self._id)
        self._alive_pub.publish(msg)

    def _clock_callback(self, time):
        return

    def update_possible_gaits(self):
        try:
            self.gait_future = self._possible_gait_client.call_async(PossibleGaits.Request())

        except Exception:
            self._node.get_logger().warn('Failed to contact get_possible_gaits service')
            return []

    def get_possible_gaits(self):
        """
        Returns a list of names of possible gaits.

        :rtype: list(str)
        :return: List of possible gaits
        """
        if self.gait_future.done():
            return self.gait_future.result().gaits
        else:
            return []

    def publish_increment_step_size(self):
        self._node.get_logger().debug('Mock Input Device published step size increment')
        self._instruction_gait_pub.publish(GaitInstruction(header=Header(stamp=self._node.get_clock().now().to_msg()),
                                                            type=GaitInstruction.INCREMENT_STEP_SIZE,
                                                            gait_name='', id=str(self._id)))
    #
    def publish_decrement_step_size(self):
        self._node.get_logger().debug('Mock Input Device published step size decrement')
        self._instruction_gait_pub.publish(GaitInstruction(header=Header(stamp=self._node.get_clock().now().to_msg()),
                                                            type=GaitInstruction.DECREMENT_STEP_SIZE,
                                                            gait_name='', id=str(self._id)))

    def publish_gait(self, string):
        self._node.get_logger().info('Mock Input Device published gait: ' + string)
        self._instruction_gait_pub.publish(GaitInstruction(header=Header(stamp=self._node.get_clock().now().to_msg()),
                                                            type=GaitInstruction.GAIT,
                                                            gait_name=string, id=str(self._id)))

    def publish_stop(self):
        msg = GaitInstruction(header=Header(stamp=self._node.get_clock().now().to_msg()),
                                            type=GaitInstruction.STOP,
                                            gait_name='', id=str(self._id))
        self._node.get_logger().info('Mock input device published stop: "{0}"'.format(msg))
        self._instruction_gait_pub.publish(msg)

    def publish_continue(self):
        self._node.get_logger().debug('Mock Input Device published continue')
        self._instruction_gait_pub.publish(GaitInstruction(header=Header(stamp=self._node.get_clock().now().to_msg()),
                                                           type=GaitInstruction.CONTINUE,
                                                           gait_name='', id=str(self._id)))

    def publish_pause(self):
        self._node.get_logger().debug('Mock Input Device published pause')
        self._instruction_gait_pub.publish(GaitInstruction(header=Header(stamp=self._node.get_clock().now().to_msg()),
                                                           type=GaitInstruction.PAUSE,
                                                           gait_name='', id=str(self._id)))

    def publish_error(self):
        self._node.get_logger().debug('Mock Input Device published error')
        self._error_pub.publish(Error(header=Header(stamp=self._node.get_clock().now().to_msg()),
                                      error_message='Fake error thrown by the develop input device.',
                                      type=Error.FATAL))

    def publish_sm_to_unknown(self):
        self._node.get_logger().debug('Mock Input Device published state machine to unknown')
        self._instruction_gait_pub.publish(GaitInstruction(header=Header(stamp=self._node.get_clock().now().to_msg()),
                                            type=GaitInstruction.UNKNOWN,
                                            gait_name='', id=str(self._id)))
