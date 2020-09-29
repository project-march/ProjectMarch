
import os

from qt_gui.plugin import Plugin
import rospkg

from .gait_selection_controller import GaitSelectionController
from .gait_selection_view import GaitSelectionView


class GaitSelectionPlugin(Plugin):
    def __init__(self, context):
        """Initiating the viewer and controller for the gait selection interface."""
        super(GaitSelectionPlugin, self).__init__(context)

        self.setObjectName('GaitSelectionPlugin')

        ui_file = os.path.join(rospkg.RosPack().get_path('march_rqt_gait_selection'), 'resource', 'gait_selection.ui')

        self._controller = GaitSelectionController()
        self._widget = GaitSelectionView(ui_file, self._controller)
        context.add_widget(self._widget)

        if context.serial_number() > 1:
            self._widget.setWindowTitle('{0} ({1})'.format(self._widget.windowTitle(), context.serial_number()))
