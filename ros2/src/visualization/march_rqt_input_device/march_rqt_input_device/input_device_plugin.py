import os
from qt_gui.plugin import Plugin
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rqt_gui.main import Main
import sys
import rclpy
from .input_device_controller import InputDeviceController
from .input_device_view import InputDeviceView


def main(args=None):
    """
    The main function used to start up the rqt input device.
    """
    rclpy.init(args=args)

    try:
        plugin = "rqt_input_device"
        main_plugin = Main(filename=plugin)
        sys.exit(main_plugin.main(standalone=plugin))

    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


class InputDevicePlugin(Plugin):
    """
    The plugin used by RQT to start the input device. This gives a context with the node to use.
    """

    def __init__(self, context):
        super(InputDevicePlugin, self).__init__(context)

        self.setObjectName("InputDevicePlugin")

        ui_file = os.path.join(
            get_package_share_directory("march_rqt_input_device"), "input_device.ui"
        )

        self._node: Node = context.node
        self._node.declare_parameter("ping_safety_node")
        self._node.declare_parameter("layout_file")
        layout_file = (
            self._node.get_parameter("layout_file").get_parameter_value().string_value
        )
        self._controller = InputDeviceController(self._node)
        self._widget = InputDeviceView(
            ui_file, layout_file, self._controller, self._node.get_logger()
        )
        context.add_widget(self._widget)

        # Show _widget.windowTitle on left-top of each plugin (when it's set in _widget). (useful for multiple windows)
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                "{0} ({1})".format(self._widget.windowTitle(), context.serial_number())
            )
