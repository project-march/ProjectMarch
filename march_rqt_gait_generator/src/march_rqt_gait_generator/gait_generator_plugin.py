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

        self.gait_publisher = None
        self.set_topic_name(self.view.topic_name_line_edit.text())

        self.view.publish_gait_button.clicked.connect(self.publish_gait)
        self.view.topic_name_line_edit.textChanged.connect(self.set_topic_name)

    def shutdown_plugin(self):
        self.controller.stop_time_slider_thread()

    def publish_gait(self, trajectory):
        rospy.loginfo('Publishing trajectory to topic ' + self.topic_name)
        trajectory = self.controller.subgait._to_joint_trajectory_msg()
        self.gait_publisher.publish(trajectory)

    def set_topic_name(self, topic_name):
        self.topic_name = topic_name
        self.gait_publisher = rospy.Publisher(topic_name, JointTrajectory, queue_size=10)
