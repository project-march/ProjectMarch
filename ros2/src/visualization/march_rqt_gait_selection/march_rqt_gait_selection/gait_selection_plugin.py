import argparse
import os
import sys
from qt_gui.plugin import Plugin
import rclpy
from rclpy.node import Node
from .gait_selection_controller import GaitSelectionController
from .gait_selection_view import GaitSelectionView
from rqt_gui.main import Main
from ament_index_python import get_package_share_directory


def main(args=None):
    """The main function used to start up the rqt note taker."""
    rclpy.init(args=args)

    try:
        plugin = 'march_rqt_gait_selection'
        main_plugin = Main(filename=plugin)
        sys.exit(main_plugin.main(
            standalone=plugin,
            plugin_argument_provider=GaitSelectionPlugin.add_arguments))

    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


class GaitSelectionPlugin(Plugin):
    def __init__(self, context):
        """Initiating the viewer and controller for the gait selection interface."""
        super(GaitSelectionPlugin, self).__init__(context)
        self._node: Node = context.node
        self.setObjectName('GaitSelectionPlugin')

        parser = argparse.ArgumentParser(prog='rqt_plot', add_help=False)
        GaitSelectionPlugin.add_arguments(parser)
        args = parser.parse_args(context.argv())

        ui_file = os.path.join(
            get_package_share_directory('march_rqt_gait_selection'), 'gait_selection.ui')

        self._controller = GaitSelectionController(self._node, source_dir=str(args.source_dir[0]))
        self._widget = GaitSelectionView(ui_file, self._controller)
        context.add_widget(self._widget)

        if context.serial_number() > 1:
            self._widget.setWindowTitle('{0} ({1})'.format(
                self._widget.windowTitle(), context.serial_number()))

    @staticmethod
    def add_arguments(parser: argparse.ArgumentParser) -> None:
        """
        Add the available arguments for the input device to the parser
        :param parser: The argument parser that is used
        """
        group = parser.add_argument_group('Options for RQT version tool')
        group.add_argument('source_dir', nargs=1,
                           help='The source home to find the march gait files')