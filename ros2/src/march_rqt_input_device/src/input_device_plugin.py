import argparse
import os
from qt_gui.plugin import Plugin
from ament_index_python.packages import get_package_share_directory
from rqt_gui.main import Main
import sys
import rclpy
from rclpy.parameter import Parameter
from .input_device_controller import InputDeviceController
from .input_device_view import InputDeviceView


def main(args=None):
    """
    The main function used to start up the rqt input device.
    """
    rclpy.init(args=args)

    try:
        plugin = 'rqt_input_device'
        main_plugin = Main(filename=plugin)
        sys.exit(main_plugin.main(standalone=plugin, plugin_argument_provider=InputDevicePlugin.add_arguments))

    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


class InputDevicePlugin(Plugin):
    """
    The plugin used by RQT to start the input device. This gives a context with the node to use.
    """
    def __init__(self, context):
        super(InputDevicePlugin, self).__init__(context)

        self.setObjectName('InputDevicePlugin')

        ui_file = os.path.join(get_package_share_directory('march_rqt_input_device_ros2'), 'input_device.ui')

        self._node = context.node

        parser = argparse.ArgumentParser(prog='rqt_plot', add_help=False)
        InputDevicePlugin.add_arguments(parser)
        args = parser.parse_args(context.argv())

        self._node.set_parameters([Parameter('use_sim_time', value=bool(args.use_sim_time))])
        self._controller = InputDeviceController(self._node, bool(args.ping_safety_node))
        self._widget = InputDeviceView(ui_file, self._controller)
        context.add_widget(self._widget)

        # Show _widget.windowTitle on left-top of each plugin (when it's set in _widget). (useful for multiple windows)
        if context.serial_number() > 1:
            self._widget.setWindowTitle('{0} ({1})'.format(self._widget.windowTitle(), context.serial_number()))

    @staticmethod
    def add_arguments(parser):
        group = parser.add_argument_group('Options for input device')
        group.add_argument('ping_safety_node', nargs=1, help='Whether to ping the safety node')
        group.add_argument('use_sim_time', nargs=1, help='Whether to use simulation time')