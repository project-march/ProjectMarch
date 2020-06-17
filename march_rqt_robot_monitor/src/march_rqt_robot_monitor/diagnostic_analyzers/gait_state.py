from diagnostic_msgs.msg import DiagnosticStatus
import rospy

from march_shared_resources.msg import GaitActionGoal


class CheckGaitStatus(object):
    def __init__(self, updater):
        """Initializes an gait diagnostic which analyzes gait and subgait states.

        :type updater: diagnostic_updater.Updater
        """
        self._updater = updater

        self._goal_sub = rospy.Subscriber('/march/gait/schedule/goal', GaitActionGoal, self._cb_goal)

    def _cb_goal(self, msg):
        """Callback for the gait scheduler goal.

        :param msg: GaitGoal
        """
        diagnostic = self._diagnostics(msg.goal.name, msg.goal.current_subgait)
        self._updater.add('(Sub)gait status', diagnostic)

    @staticmethod
    def _diagnostics(name, subgait):
        """Create a diagnostic function corresponding to gait and subgait data.

        :param name: the gait name
        :param subgait: the subgait object from the GaitGoal message
        """
        def d(stat):
            stat.add('Gait type', subgait.gait_type)
            stat.add('Subgait version', subgait.version)
            stat.summary(DiagnosticStatus.OK, '{gait}, {subgait}'.format(gait=name, subgait=subgait.name))

            return stat

        return d
