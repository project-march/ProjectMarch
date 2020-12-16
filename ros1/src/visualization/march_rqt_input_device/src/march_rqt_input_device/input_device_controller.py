import getpass
import socket

import rospy
from std_msgs.msg import Header

from march_shared_msgs.msg import Alive, CurrentState, Error, GaitInstruction, GaitInstructionResponse
from march_shared_msgs.srv import PossibleGaits


class InputDeviceController(object):

    # Format of the identifier for the alive message
    ID_FORMAT = 'rqt@{machine}@{user}'

    def __init__(self, ping):
        self._instruction_gait_pub = rospy.Publisher('/march/input_device/instruction', GaitInstruction, queue_size=10)
        self._instruction_response_pub = rospy.Subscriber('/march/input_device/instruction_response',
                                                          GaitInstructionResponse, callback=self._response_callback,
                                                          queue_size=10)
        self._current_gait = rospy.Subscriber('/march/gait_selection/current_state', CurrentState,
                                              callback=self._current_state_cb, queue_size=1)
        self._error_pub = rospy.Publisher('/march/error', Error, queue_size=10)
        self._get_possible_gaits = rospy.ServiceProxy('/march/gait_selection/get_possible_gaits', PossibleGaits)

        self.accepted_cb = None
        self.finished_cb = None
        self.rejected_cb = None
        self.current_gait_cb = None

        self._ping = ping
        self._id = self.ID_FORMAT.format(machine=socket.gethostname(),
                                         user=getpass.getuser())

        if self._ping:
            self._alive_pub = rospy.Publisher('/march/input_device/alive', Alive, queue_size=10)
            period = rospy.Duration().from_sec(0.2)
            self._alive_timer = rospy.Timer(period, self._timer_callback)

    def __del__(self):
        self._instruction_gait_pub.unregister()
        self._error_pub.unregister()
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

    def _current_state_cb(self, msg):
        if callable(self.current_gait_cb):
            self.current_gait_cb(msg.state)

    def _timer_callback(self, event):
        msg = Alive(stamp=event.current_real, id=self._id)
        self._alive_pub.publish(msg)

    def get_possible_gaits(self):
        """
        Returns a list of names of possible gaits.

        :rtype: list(str)
        :return: List of possible gaits
        """
        try:
            return self._get_possible_gaits().gaits
        except rospy.ServiceException:
            rospy.logwarn('Failed to contact get_possible_gaits service')
            return []

    def publish_increment_step_size(self):
        rospy.logdebug('Mock Input Device published step size increment')
        self._instruction_gait_pub.publish(GaitInstruction(Header(stamp=rospy.Time.now()),
                                                           GaitInstruction.INCREMENT_STEP_SIZE, '', self._id))

    def publish_decrement_step_size(self):
        rospy.logdebug('Mock Input Device published step size decrement')
        self._instruction_gait_pub.publish(GaitInstruction(Header(stamp=rospy.Time.now()),
                                                           GaitInstruction.DECREMENT_STEP_SIZE, '', self._id))

    def publish_gait(self, string):
        rospy.logdebug('Mock Input Device published gait: ' + string)
        self._instruction_gait_pub.publish(GaitInstruction(Header(stamp=rospy.Time.now()),
                                                           GaitInstruction.GAIT, string, self._id))

    def publish_stop(self):
        rospy.logdebug('Mock Input Device published stop')
        self._instruction_gait_pub.publish(GaitInstruction(Header(stamp=rospy.Time.now()),
                                                           GaitInstruction.STOP, '', self._id))

    def publish_continue(self):
        rospy.logdebug('Mock Input Device published continue')
        self._instruction_gait_pub.publish(GaitInstruction(Header(stamp=rospy.Time.now()),
                                                           GaitInstruction.CONTINUE, '', self._id))

    def publish_pause(self):
        rospy.logdebug('Mock Input Device published pause')
        self._instruction_gait_pub.publish(GaitInstruction(Header(stamp=rospy.Time.now()),
                                                           GaitInstruction.PAUSE, '', self._id))

    def publish_error(self):
        rospy.logdebug('Mock Input Device published error')
        self._error_pub.publish(Error(Header(stamp=rospy.Time.now()),
                                      'Fake error thrown by the develop input device.', Error.FATAL))

    def publish_sm_to_unknown(self):
        rospy.logdebug('Mock Input Device published state machine to unknown')
        self._instruction_gait_pub.publish(GaitInstruction(Header(stamp=rospy.Time.now()),
                                                           GaitInstruction.UNKNOWN, '', self._id))
