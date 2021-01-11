from diagnostic_msgs.msg import DiagnosticStatus
import rospy

from march_shared_msgs.msg import ImcState


class CheckImcStatus:
    def __init__(self, updater):
        """Initializes an IMC diagnostic which analyzes IMC states.

        :type updater: diagnostic_updater.Updater
        """
        self._updater = updater
        self._sub = rospy.Subscriber("/march/imc_states", ImcState, self._cb)
        self._imc_state = None

        self._diagnostics = set()

    def _cb(self, msg):
        """Callback for imc_states.

        :type msg: ImcState
        """
        self._imc_state = msg
        for i in range(len(msg.joint_names)):
            joint_name = msg.joint_names[i]
            if joint_name not in self._diagnostics:
                self._diagnostics.add(joint_name)
                self._updater.add("IMC {0}".format(joint_name), self._diagnostic(i))

    def _diagnostic(self, index):
        """Creates a diagnostic function for an IMC.

        :type index: int
        :param index: index of the joint
        :return Curried diagnostic function that updates the diagnostic status
                according to the given index
        """

        def d(stat):
            if self._imc_state.joint_names[index] is None:
                stat.summary(DiagnosticStatus.STALE, "No more events recorded")
                return stat
            detailed_error = int(self._imc_state.detailed_error[index])
            motion_error = int(self._imc_state.motion_error[index])
            state = self._imc_state.state[index]

            stat.add("Status word", self._imc_state.status_word[index])
            if detailed_error != 0 or motion_error != 0:
                stat.add("Detailed error", self._imc_state.detailed_error[index])
                stat.add(
                    "Detailed error description",
                    self._imc_state.detailed_error_description[index],
                )
                stat.add("Motion error", self._imc_state.motion_error[index])
                stat.add(
                    "Motion error description",
                    self._imc_state.motion_error_description[index],
                )
                stat.summary(DiagnosticStatus.ERROR, state)
            else:
                stat.summary(DiagnosticStatus.OK, state)

            return stat

        return d
