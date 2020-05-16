
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import \
    QComboBox, QDialog, QGridLayout, QLabel, QPlainTextEdit, QPushButton, QScrollArea, QSizePolicy, QWidget
from python_qt_binding import loadUi
import rospy


AMOUNT_OF_AVAILABLE_SUBGAITS = 5


class GaitSelectionView(QWidget):
    def __init__(self, ui_file, controller):
        """Base class to load and use the gait_selection.ui.

        :param ui_file:
            Path to the .ui file to load with the widget
        :param controller:
            A refrence to the controller corresponding to the rqt gait selection functionalities
        """
        super(GaitSelectionView, self).__init__(flags=Qt.Window)

        self._controller = controller

        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self)

        # Coding flags
        self._colors = {'info': '#000000', 'success': '#009100', 'error': '#FF0000', 'warning': '#b27300'}
        self._is_refresh_active = True
        self._is_update_active = True

        # Search for children in GUI
        self._gait_content = self.Gaits.findChild(QScrollArea, 'GaitSelectionInterface')
        self._gait_menu = self._gait_content.findChild(QComboBox, 'GaitMenu')
        self._logger = self._gait_content.findChild(QPlainTextEdit, 'Log')

        self._subgait_labels = []
        self._subgait_menus = []

        for index in range(AMOUNT_OF_AVAILABLE_SUBGAITS):
            self._subgait_labels.append(self._gait_content.findChild(QLabel, 'SubgaitLabel_{nr}'.format(nr=index)))
            self._subgait_menus.append(self._gait_content.findChild(QComboBox, 'SubgaitMenu_{nr}'.format(nr=index)))

        # bind functions to callbacks of buttons and menus
        self.findChild(QPushButton, 'Refresh').clicked.connect(lambda: self._refresh())
        self.findChild(QPushButton, 'Apply').clicked.connect(lambda: [self._apply(), self._refresh()])
        self.findChild(QPushButton, 'SaveDefault').clicked.connect(lambda: self._save_default())
        self.findChild(QPushButton, 'ClearLogger').clicked.connect(lambda: self._logger.clear())
        self.findChild(QPushButton, 'SeeAllVersions').clicked.connect(lambda: self._show_version_map_pop_up())

        self._gait_menu.currentIndexChanged.connect(lambda: self.update_version_menus())

        for subgait_menu in self._subgait_menus:
            subgait_menu.currentIndexChanged.connect(lambda: self.update_version_colors())

        # loaded gaits from gait selection node
        self.available_gaits = {}
        self.version_map = {}

        # pop up window
        self._version_map_pop_up = PopUpWindow(self)

        # populate gait menu for the first time
        self._refresh()

    # gait and subgait related layout functions
    def update_version_menus(self):
        """When a gait is selected set the subgait labels and populate the subgait menus with the available versions."""
        self._is_update_active = True
        if self._is_refresh_active:
            return

        self._clear_gui()

        gait_name = self._gait_menu.currentText()
        subgaits = self.available_gaits[gait_name]['subgaits']

        latest_used_index = 0
        for index, (subgait_name, versions) in enumerate(subgaits.items()):

            try:
                subgait_menu = self._subgait_menus[index]
                subgait_label = self._subgait_labels[index]
            except index:
                rospy.logfatal('Not enough labels and menus available in GUI, change layout using Qt tool.')
                return

            subgait_menu.setDisabled(0)
            subgait_label.setDisabled(0)

            subgait_label.setText(subgait_name)
            subgait_menu.addItems(versions)

            try:
                current_version = self.version_map[gait_name][subgait_name]
                current_version_index = versions.index(current_version)
                subgait_menu.setCurrentIndex(current_version_index)

            except ValueError:
                self._log('Default version of {gn} {sgn} does not exist in loaded gaits.'
                          .format(gn=gait_name, sgn=subgait_name), 'error')
            except KeyError:
                self._log('{gn} has no default version for {sgn}.'
                          .format(gn=gait_name, sgn=subgait_name), 'error')

            latest_used_index = index + 1

        for unused_index in range(latest_used_index, AMOUNT_OF_AVAILABLE_SUBGAITS):
            self._subgait_labels[unused_index].setDisabled(1)
            self._subgait_menus[unused_index].setDisabled(1)

        self._is_update_active = False

    def update_selected_versions(self):
        """Read the subgait version menus and save the newly selected versions to the local version map."""
        gait_name = self._gait_menu.currentText()

        for subgait_label, subgait_menu in zip(self._subgait_labels, self._subgait_menus):
            subgait_name = subgait_label.text()
            if subgait_name != 'Unused':
                try:
                    self.version_map[gait_name][subgait_name] = subgait_menu.currentText()
                except KeyError:
                    pass

    def update_version_colors(self):
        """Update the subgait labels with a specific color to represent a change in version."""
        if self._is_update_active or self._is_refresh_active:
            return

        gait_name = self._gait_menu.currentText()
        for subgait_label, subgait_menu in zip(self._subgait_labels, self._subgait_menus):
            subgait_name = subgait_label.text()
            if subgait_name != 'Unused':
                try:
                    if str(self.version_map[gait_name][subgait_name]) != str(subgait_menu.currentText()):
                        subgait_label.setStyleSheet('color:{color}'.format(color=self._colors['warning']))
                    else:
                        subgait_label.setStyleSheet('color:{color}'.format(color=self._colors['info']))
                except KeyError:
                    pass

    def _refresh(self):
        """Request the gait map from the gait selection node and display the available gaits in the gait menu."""
        self._log('Starting refresh', 'warning')

        self._is_refresh_active = True
        self._clear_gui(clear_gait_menu=True)

        self.available_gaits = self._controller.get_directory_structure()
        self.version_map = self._controller.get_version_map()

        self._gait_menu.addItems(sorted(self.available_gaits.keys()))

        self._log('Directory data refreshed', 'success')
        self._is_refresh_active = False

    # logger
    def _log(self, msg, color_tag='info'):
        """Use the logger window in the GUI to display data.

        :param msg:
            The message to display in the widget
        :param color_tag:
            The tag which represents the color of the text in the screen (info, warning, error)
        """
        try:
            color = self._colors[color_tag]
        except KeyError:
            color = self._colors['error']

        self._logger.appendHtml('<p style="color:' + str(color) + '">' + msg + '</p>')
        scrollbar = self.findChild(QPlainTextEdit, 'Log').verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    # button functions
    def _clear_gui(self, clear_gait_menu=False):
        """Clear the subgait menus and subgait labels and optionally the gait menu.

        :param clear_gait_menu:
            Set this to true if the gait menu should also be cleared
        """
        for subgait_menu in self._subgait_menus:
            subgait_menu.clear()

        for subgait_label in self._subgait_labels:
            subgait_label.setStyleSheet('color:#000000')
            subgait_label.setText('Unused')

        if clear_gait_menu:
            self._gait_menu.clear()

    def _save_default(self):
        """Save the currently selected subgait versions as default values."""
        self._log('Starting saving default', 'warning')
        if self._controller.set_default_versions():
            self._log('Set new default versions', 'success')
        else:
            self._log('Set new default versions failed', 'error')

    def _apply(self):
        """Apply newly selected subgait versions to the gait selection node."""
        self._log('Starting apply', 'warning')
        self.update_selected_versions()
        if self._controller.set_version_map(self.version_map):
            self._log('Version change applied', 'success')
        else:
            self._log('Version change applied failed', 'failed')

    def _show_version_map_pop_up(self):
        """Use a pop up window to display all the gait, subgaits and currently used versions."""
        version_map = self._controller.get_version_map()
        version_map_string = ''

        for gait_name in sorted(version_map.keys()):
            version_map_string += '{gait} \n'.format(gait=gait_name)
            for subgait_name, version in version_map[gait_name].items():
                version_map_string += '\t{sb:<30} \t {vs} \n'.format(sb=subgait_name, vs=version)
            version_map_string += '\n'

        self._version_map_pop_up.show_message(version_map_string)


class PopUpWindow(QDialog):
    def __init__(self, parent, width=500, height=600):
        """Base class for creating a pop up window over an existing widget.

        :param parent:
            The parent widget to connect to the pop up
        :param width:
            Starting width of the the pop up widget
        :param height:
            Starting height of the the pop up widget
        """
        super(PopUpWindow, self).__init__(parent=parent, flags=Qt.Window)
        self.resize(width, height)
        self.setSizePolicy(QSizePolicy.Minimum, QSizePolicy.Minimum)

        self._layout = QGridLayout(self)

        self._scroll_area = QScrollArea()
        self._scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self._scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self._scroll_area.setSizePolicy(QSizePolicy.Minimum, QSizePolicy.Minimum)
        self._scroll_area.setWidgetResizable(True)

        self._content_frame = QWidget(self._scroll_area, flags=Qt.Window)
        self._content_frame.setSizePolicy(QSizePolicy.Minimum, QSizePolicy.Minimum)
        self._content = QGridLayout(self._content_frame)

        self.msg_label = QLabel(self)
        self._content.addWidget(self.msg_label)

        self._scroll_area.setWidget(self._content_frame)
        self._layout.addWidget(self._scroll_area)

    def show_message(self, message):
        """Add message to the pop up and show the window."""
        self.msg_label.clear()
        self.msg_label.setText(message)
        return super(PopUpWindow, self).show()
