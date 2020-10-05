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
    rclpy.init(args=args)

    try:
        plugin = 'rqt_input_device'
        main_plugin = Main(filename=plugin)
        sys.exit(main_plugin.main(standalone=plugin))

    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


class InputDevicePlugin(Plugin):
    def __init__(self, context):
        super(InputDevicePlugin, self).__init__(context)

        self.setObjectName('InputDevicePlugin')
        context.node.set_parameters([Parameter("use_sim_time", value=True)])

        # ping = rclpy.get_param('~ping_safety_node', True)
        ui_file = os.path.join(get_package_share_directory('march_rqt_input_device_ros2'), 'input_device.ui')

        self._node = context.node
        self._controller = InputDeviceController(self._node, True)
        # self._timesource = self.create_subscription(Clock, '/clock', self._clock_callback, 10)
        self._widget = InputDeviceView(ui_file, self._controller)
        context.add_widget(self._widget)

        # Show _widget.windowTitle on left-top of each plugin (when it's set in _widget). (useful for multiple windows)
        if context.serial_number() > 1:
            self._widget.setWindowTitle('{0} ({1})'.format(self._widget.windowTitle(), context.serial_number()))

        # rclpy.spin(self._controller)
