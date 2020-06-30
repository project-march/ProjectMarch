from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QComboBox, QLabel, QWidget
from python_qt_binding import loadUi

from .gait_selection_errors import GaitSelectionError
from .gait_selection_pop_up import PopUpWindow


DEFAULT_AMOUNT_OF_AVAILABLE_SUBGAITS = 3


class GaitSelectionView(QWidget):
    def __init__(self, ui_file, controller):
        """Base class to load and use the gait_selection.ui.

        :param ui_file:
            Path to the .ui file to load with the widget
        :param controller:
            A reference to the controller corresponding to the rqt gait selection functionality
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
        self._subgaits = self.SubgaitsFrame
        self._gait_menu = self.GaitMenu
        self._logger = self.Log

        self._subgait_labels = []
        self._subgait_menus = []

        for index in range(DEFAULT_AMOUNT_OF_AVAILABLE_SUBGAITS):
            self._subgait_labels.append(getattr(self, 'SubgaitLabel_{nr}'.format(nr=index)))
            self._subgait_menus.append(getattr(self, 'SubgaitMenu_{nr}'.format(nr=index)))

        # bind functions to callbacks of buttons and menus
        self.Refresh.pressed.connect(lambda: self._refresh())
        self.Apply.pressed.connect(lambda: [self._apply(), self._refresh()])
        self.SaveDefault.pressed.connect(lambda: [self._apply(), self._save_default(), self._refresh()])

        self.ClearLogger.clicked.connect(lambda: self._logger.clear())
        self.SeeAllVersions.clicked.connect(lambda: self._show_version_map_pop_up())
        self.LoadSubgaits.clicked.connect(lambda: self.update_version_menus())

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
    def add_subgait_menus(self, amount_of_new_subgait_menus):
        """Add subgait labels and dropdown menu's in case a gait has more subgaits then available menu's.

        :param amount_of_new_subgait_menus: the amount of new subgait labels and menu's
        """
        for new_subgait in range(amount_of_new_subgait_menus):

            new_subgait_label = QLabel(self)
            new_subgait_label.setFont(self._subgait_labels[0].font())
            new_subgait_label.setAlignment(self._subgait_labels[0].alignment())

            new_subgait_menu = QComboBox(self)
            new_subgait_menu.setFont(self._subgait_menus[0].font())
            new_subgait_menu.currentIndexChanged.connect(lambda: self.update_version_colors())

            self._subgait_labels.append(new_subgait_label)
            self._subgait_menus.append(new_subgait_menu)

            self._subgaits.layout().addRow(new_subgait_label, new_subgait_menu)

    def update_version_menus(self):
        """When a gait is selected set the subgait labels and populate the subgait menus with the available versions."""

        def version_sorter(version):
            """Used in the sort function to sort numbers which are 9<

            :param version: str of the version
            """
            try:
                length = len(version)
                version_number = ''.join(letter for letter in version[length-2: length] if letter.isdigit())
                return int(version_number)
            except ValueError:
                return version

        self._is_update_active = True
        if self._is_refresh_active:
            return

        self._clear_gui()

        gait_name = self._gait_menu.currentText()
        subgaits = self.available_gaits[gait_name]

        if len(subgaits) > len(self._subgait_labels):
            amount_of_new_subgait_menus = len(subgaits) - len(self._subgait_labels)
            self.add_subgait_menus(amount_of_new_subgait_menus)

        latest_used_index = 0

        for index, (subgait_name, versions) in enumerate(subgaits.items()):
            subgait_label = self._subgait_labels[index]
            subgait_menu = self._subgait_menus[index]

            subgait_menu.show()
            subgait_label.show()

            subgait_label.setText(subgait_name)
            subgait_menu.addItems(sorted(versions, key=version_sorter))

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

        for unused_index in range(latest_used_index, len(self._subgait_labels)):
            self._subgait_labels[unused_index].hide()
            self._subgait_menus[unused_index].hide()

        self._is_update_active = False

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
        self._is_refresh_active = True
        self._clear_gui(clear_gait_menu=True)

        try:
            self.available_gaits = self._controller.get_directory_structure()
            self.version_map = self._controller.get_version_map()

            self._gait_menu.addItems(sorted(self.available_gaits.keys()))

            self._log('Directory data refreshed', 'success')
        except GaitSelectionError as e:
            self._log(str(e), 'error')
        finally:
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
        scrollbar = self.Log.verticalScrollBar()
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
        try:
            success, msg = self._controller.set_default_versions()
            if success:
                self._log(msg if msg else 'Set new default versions', 'success')
            else:
                self._log(msg if msg else 'Failed to set default versions', 'error')
        except GaitSelectionError as e:
            self._log(str(e), 'error')

    def _apply(self):
        """Apply newly selected subgait versions to the gait selection node."""
        gait_name = self._gait_menu.currentText()
        subgait_names = []
        versions = []

        for subgait_label, subgait_menu in zip(self._subgait_labels, self._subgait_menus):
            if subgait_label.text() != 'Unused':
                subgait_names.append(subgait_label.text())
                versions.append(subgait_menu.currentText())

        try:
            success, msg = self._controller.set_gait_version(gait_name, subgait_names, versions)
            if success:
                self._log(msg if msg else 'Version change applied', 'success')
            else:
                self._log(msg if msg else 'Version change applied failed', 'failed')
        except GaitSelectionError as e:
            self._log(str(e), 'error')

    def _show_version_map_pop_up(self):
        """Use a pop up window to display all the gait, subgaits and currently used versions."""
        try:
            version_map = self._controller.get_version_map()
        except GaitSelectionError as e:
            self._log(str(e), 'error')
            return

        version_map_string = ''
        for gait_name in sorted(version_map.keys()):
            version_map_string += '{gait} \n'.format(gait=gait_name)
            for subgait_name, version in version_map[gait_name].items():
                version_map_string += '\t{sb:<30} \t {vs} \n'.format(sb=subgait_name, vs=version)
            version_map_string += '\n'

        self._version_map_pop_up.show_message(version_map_string)
