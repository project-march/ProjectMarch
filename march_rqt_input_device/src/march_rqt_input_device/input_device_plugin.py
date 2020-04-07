import os

from qt_gui.plugin import Plugin
import rospkg
import rospy

from .input_device_controller import InputDeviceController
from .input_device_view import InputDeviceView


class InputDevicePlugin(Plugin):
    def __init__(self, context):
        super(InputDevicePlugin, self).__init__(context)

        self.setObjectName('InputDevicePlugin')

        ping = rospy.get_param('~ping_safety_node', True)
        ui_file = os.path.join(rospkg.RosPack().get_path('march_rqt_input_device'), 'resource', 'input_device.ui')

        self._controller = InputDeviceController(ping)
        self._widget = InputDeviceView(ui_file, self._controller)
        context.add_widget(self._widget)

        # Show _widget.windowTitle on left-top of each plugin (when it's set in _widget). (useful for multiple windows)
        if context.serial_number() > 1:
            self._widget.setWindowTitle('{0} ({1})'.format(self._widget.windowTitle(), context.serial_number()))
