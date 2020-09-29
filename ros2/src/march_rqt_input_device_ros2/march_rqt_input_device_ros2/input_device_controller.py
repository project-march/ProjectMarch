import getpass
import socket
import rclpy
from rclpy.time import Time as RclTime
from rclpy.clock import ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Header, String
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time
from march_shared_msgs.msg import Alive, Error, GaitInstruction2, GaitInstructionResponse
from march_shared_msgs.srv import PossibleGaits

class InputDeviceController(Node):

    # Format of the identifier for the alive message
    ID_FORMAT = 'rqt@{machine}@{user}ros2'

    def __init__(self, ping):
        super().__init__('input_device_ros2')
        self._instruction_gait_pub = self.create_publisher(GaitInstruction2, '/march/input_device/instruction', 10)
        self._instruction_response_pub = self.create_subscription(GaitInstructionResponse,
                                                                  '/march/input_device/instruction_response',
                                                                  self._response_callback, 10)
        self._current_gait = self.create_subscription(String, '/march/gait/current',
                                                        self._current_gait_callback, 1)
        self._error_pub = self.create_publisher(Error, '/march/error', 10)
        self._get_possible_gaits = self.create_client(PossibleGaits, '/march/state_machine/get_possible_gaits')

        self.set_parameters([Parameter("use_sim_time", value=True)])

        self.accepted_cb = None
        self.finished_cb = None
        self.rejected_cb = None
        self.current_gait_cb = None
        self.get_logger().info(str(type(self.get_clock())))
        self.get_logger().info(str(self.get_clock()))
        self.get_logger().info(str(self.get_clock().now().nanoseconds))
        self._ping = ping
        self._id = self.ID_FORMAT.format(machine=socket.gethostname(),
                                         user=getpass.getuser())
        self.get_logger().info(str(self._ping))
        self.create_subscription(Clock, '/clock', self._clock_callback, 1)

        if self._ping:
            self.get_logger().info('Creating alive timer')
            self._alive_pub = self.create_publisher(Alive, '/march/input_device/alive', 10)
            period = 0.02 #secs
            self._alive_timer = self.create_timer(period, self._timer_callback)
            msg = Alive(stamp=self.get_clock().now().to_msg(), id=self._id)
            self._alive_pub.publish(msg)
            self.get_logger().info('Published alive timer')

    def __del__(self):
        self.destroy_publisher(self._instruction_gait_pub)
        self.destroy_publisher(self._error_pub)
        if self._ping:
            self._alive_timer.shutdown()
            self._alive_timer.join()
            self._alive_pub.unregister()

    def _clock_callback(self, msg):
        return
        # self.get_logger().info(msg)
        # self.get_clock().set_ros_time_override(RclTime.from_msg(msg), clock_type=ClockType.ROS_TIME)

    def _callback_test(self):
        self.get_logger().info('TEST')

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

    def _timer_callback(self, event):
        self.get_logger().info(str(type(event)))
        msg = Alive(stamp=self.get_clock.now().to_msg(), id=self._id)
        self.get_logger().info('Sending ALIVE: {0}'.format(self.get_clock.now()))
        self._alive_pub.publish(msg)

    def get_possible_gaits(self):
        """
        Returns a list of names of possible gaits.

        :rtype: list(str)
        :return: List of possible gaits
        """
        # try:
        #     return self._get_possible_gaits().call(PossibleGaits.Request())
        # except Exception:
        #     self.get_logger().warn('Failed to contact get_possible_gaits service')
        return []

    def _default_callback_group(self, msg):
        self.get_logger().info(msg)
        return

    def publish_increment_step_size(self):
        self.get_logger().debug('Mock Input Device published step size increment')
        self._instruction_gait_pub.publish(GaitInstruction2(header=Header(stamp=self._get_time()),
                                                            type=GaitInstruction2.INCREMENT_STEP_SIZE,
                                                            gait_name='', id=str(self._id)))
    #
    def publish_decrement_step_size(self):
        self.get_logger.debug('Mock Input Device published step size decrement')
        self._instruction_gait_pub.publish(GaitInstruction2(header=Header(stamp=self._get_time()),
                                                            type=GaitInstruction2.DECREMENT_STEP_SIZE,
                                                            gait_name='', id=str(self._id)))

    def publish_gait(self, string):
        self.get_logger().debug('Mock Input Device published gait: ' + string)
        self._instruction_gait_pub.publish(GaitInstruction2(header=Header(stamp=self._get_time()),
                                                            type=GaitInstruction2.GAIT,
                                                            gait_name=string, id=str(self._id)))

    def publish_stop(self):
        msg = GaitInstruction2(header=Header(stamp=self._get_time()), type=GaitInstruction2.STOP, gait_name='', id=str(self._id))
        self.get_logger().info('Publishing: "{0}"'.format(msg))
        self._instruction_gait_pub.publish(msg)

    # def publish_continue(self):
    #     rospy.logdebug('Mock Input Device published continue')
    #     self._instruction_gait_pub.publish(GaitInstruction2(Header(stamp=rospy.Time.now()),
    #                                                        GaitInstruction.CONTINUE, '', self._id))
    #
    # def publish_pause(self):
    #     rospy.logdebug('Mock Input Device published pause')
    #     self._instruction_gait_pub.publish(GaitInstruction2(Header(stamp=rospy.Time.now()),
    #                                                        GaitInstruction.PAUSE, '', self._id))
    #
    # def publish_error(self):
    #     rospy.logdebug('Mock Input Device published error')
    #     self._error_pub.publish(Error(Header(stamp=rospy.Time.now()),
    #                                   'Fake error thrown by the develop input device.', Error.FATAL))
    #
    # def publish_sm_to_unknown(self):
    #     rospy.logdebug('Mock Input Device published state machine to unknown')
    #     self._instruction_gait_pub.publish(GaitInstruction2(Header(stamp=rospy.Time.now()),
    #                                                        GaitInstruction.UNKNOWN, '', self._id))
