from qt_gui.plugin import Plugin
import rospy
from trajectory_msgs.msg import JointTrajectory
from urdf_parser_py import urdf

from .gait_generator_controller import GaitGeneratorController
from .gait_generator_view import GaitGeneratorView


class GaitGeneratorPlugin(Plugin):
    def __init__(self, context):
        super(GaitGeneratorPlugin, self).__init__(context)

        self.view = GaitGeneratorView()
        context.add_widget(self.view)

        robot = urdf.Robot.from_parameter_server()
        self.controller = GaitGeneratorController(self.view, robot)

    def shutdown_plugin(self):
        self.controller.stop_time_slider_thread()
