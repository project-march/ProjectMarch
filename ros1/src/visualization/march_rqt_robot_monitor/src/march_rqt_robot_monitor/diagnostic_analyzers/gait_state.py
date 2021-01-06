from diagnostic_msgs.msg import DiagnosticStatus
import rospy

from march_shared_msgs.msg import CurrentGait


class CheckGaitStatus(object):
    def __init__(self, updater):
        """Initializes an gait diagnostic which analyzes gait and subgait states.

        :type updater: diagnostic_updater.Updater
        """
        self._goal_sub = rospy.Subscriber('/march/gait_selection/current_gait', CurrentGait, self._cb_goal)
        self._gait_msg = None

        self._updater = updater
        self._updater.add('Gait', self._diagnostics)

    def _cb_goal(self, msg):
        """Callback for the gait scheduler goal.

        :param msg: GaitGoal
        """
        self._gait_msg = msg

    def _diagnostics(self, stat):
        """Create a diagnostic function corresponding to gait and subgait data."""
        if self._gait_msg is None:
            stat.add('Topic error', 'No events recorded')
            stat.summary(DiagnosticStatus.STALE, 'No gait activity recorded')
            return stat

        stat.add('Gait', str(self._gait_msg.gait))
        stat.add('Subgait', str(self._gait_msg.subgait))
        stat.add('Subgait version', str(self._gait_msg.version))
        stat.add('Gait type', str(self._gait_msg.gait_type))

        stat.summary(DiagnosticStatus.OK, 'Gait: {gait}, {subgait}'
                     .format(gait=str(self._gait_msg.gait), subgait=str(self._gait_msg.subgait)))

        return stat
