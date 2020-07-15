from diagnostic_msgs.msg import DiagnosticStatus
import rospy

from march_shared_resources.msg import GaitActionGoal


class CheckGaitStatus(object):
    def __init__(self, updater):
        """Initializes an gait diagnostic which analyzes gait and subgait states.

        :type updater: diagnostic_updater.Updater
        """
        self._goal_sub = rospy.Subscriber('/march/gait/schedule/goal', GaitActionGoal, self._cb_goal)
        self._goal_msg = None

        self._updater = updater
        self._updater.add('Gait', self._diagnostics)

    def _cb_goal(self, msg):
        """Callback for the gait scheduler goal.

        :param msg: GaitGoal
        """
        self._goal_msg = msg

    def _diagnostics(self, stat):
        """Create a diagnostic function corresponding to gait and subgait data."""
        if self._goal_msg is None:
            stat.add('Topic error', 'No events recorded')
            stat.summary(DiagnosticStatus.STALE, 'No gait activity recorded')
            return stat

        stat.add('Gait', str(self._goal_msg.goal.gait_name))
        stat.add('Subgait', str(self._goal_msg.goal.subgait_name))
        stat.add('Gait type', str(self._goal_msg.goal.gait_type))
        stat.add('Subgait version', str(self._goal_msg.goal.version))

        stat.summary(DiagnosticStatus.OK, 'Gait: {gait}, {subgait}'
                     .format(gait=str(self._goal_msg.goal.gait_name), subgait=str(self._goal_msg.goal.subgait_name)))

        return stat
