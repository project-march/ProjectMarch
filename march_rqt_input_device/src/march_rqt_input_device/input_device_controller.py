import rospy
import std_msgs.msg

from march_shared_resources.msg import Error, GaitInstruction


class InputDeviceController(object):
    def __init__(self):
        self.instruction_gait_pub = rospy.Publisher('/march/input_device/instruction', GaitInstruction, queue_size=10)

        self.error_pub = rospy.Publisher('/march/error', Error, queue_size=10)
        self._ping = rospy.get_param('~ping_safety_node', True)

        if self._ping:
            self._alive_pub = rospy.Publisher('/march/input_device/alive', std_msgs.msg.Time, queue_size=10)
            period = rospy.Duration().from_sec(0.2)
            self._alive_timer = rospy.Timer(period, self._timer_callback)

    def shutdown_plugin(self):
        if self._ping:
            self._alive_timer.shutdown()
            self._alive_pub.unregister()

    def _timer_callback(self, event):
        self._alive_pub.publish(event.current_real)

    def publish_increment_step_size(self):
        rospy.logdebug('Mock Input Device published step size increment')
        self.instruction_gait_pub.publish(GaitInstruction(std_msgs.msg.Header(stamp=rospy.Time.now()),
                                                          GaitInstruction.INCREMENT_STEP_SIZE, ''))

    def publish_decrement_step_size(self):
        rospy.logdebug('Mock Input Device published step size decrement')
        self.instruction_gait_pub.publish(GaitInstruction(std_msgs.msg.Header(stamp=rospy.Time.now()),
                                                          GaitInstruction.DECREMENT_STEP_SIZE, ''))

    def publish_gait(self, string):
        rospy.logdebug('Mock Input Device published gait: ' + string)
        self.instruction_gait_pub.publish(GaitInstruction(std_msgs.msg.Header(stamp=rospy.Time.now()),
                                                          GaitInstruction.GAIT, string))

    def publish_stop(self):
        rospy.logdebug('Mock Input Device published stop')
        self.instruction_gait_pub.publish(GaitInstruction(std_msgs.msg.Header(stamp=rospy.Time.now()),
                                                          GaitInstruction.STOP, ''))

    def publish_continue(self):
        rospy.logdebug('Mock Input Device published continue')
        self.instruction_gait_pub.publish(GaitInstruction(std_msgs.msg.Header(stamp=rospy.Time.now()),
                                                          GaitInstruction.CONTINUE, ''))

    def publish_pause(self):
        rospy.logdebug('Mock Input Device published pause')
        self.instruction_gait_pub.publish(GaitInstruction(std_msgs.msg.Header(stamp=rospy.Time.now()),
                                                          GaitInstruction.PAUSE, ''))

    def publish_error(self):
        rospy.logdebug('Mock Input Device published error')
        self.error_pub.publish(Error(std_msgs.msg.Header(stamp=rospy.Time.now()),
                                     'Fake error thrown by the develop input device.', Error.FATAL))
