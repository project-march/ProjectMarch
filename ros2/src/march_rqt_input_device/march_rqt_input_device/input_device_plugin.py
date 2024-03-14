"""Author: Olav de Haas MIV & Andrew Hutani, MIX"""
import os
import json
from typing import List, Callable, Tuple, Optional, Union
from qt_gui.plugin import Plugin
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rqt_gui.main import Main
import sys
import rclpy
from .input_device_controller import InputDeviceController
from .input_device_view import InputDeviceView
import python_qt_binding.QtWidgets as QtWidgets
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QPushButton
from PyQt5.QtWidgets import QToolButton


def main(args=None):
    """The main function used to start up the rqt input device."""
    rclpy.init(args=args)

    try:
        plugin = "rqt_input_device"
        main_plugin = Main(filename=plugin)
        sys.exit(main_plugin.main(standalone=plugin))

    except KeyboardInterrupt:
        pass

    rclpy.shutdown()



class InputDevicePlugin(Plugin):
    def __init__(self, context):
        super(InputDevicePlugin, self).__init__(context)
        self.setObjectName("rqt_input_device")

        ui_file = os.path.join(get_package_share_directory("march_rqt_input_device"), "input_device.ui")
        layout_file = os.path.join(get_package_share_directory("march_rqt_input_device"), "modes.json")

        # layout_file = os.path.join("src/march_mode_machine/", "generate", "modes.json")

        self._node: Node = context.node

        self._controller = InputDeviceController(self._node)
        self._widget = InputDeviceView(ui_file, self._controller, layout_file)

        self._controller.set_view(self._widget)
        
        context.add_widget(self._widget)

        if context.serial_number() > 1:
            self._widget.setWindowTitle("{0} ({1})".format(self._widget.windowTitle(), context.serial_number()))

