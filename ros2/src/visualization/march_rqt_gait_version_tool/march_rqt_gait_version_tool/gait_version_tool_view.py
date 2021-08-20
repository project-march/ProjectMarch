import re
from enum import Enum

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QComboBox, QLabel, QWidget
from python_qt_binding import loadUi

from .gait_version_tool_errors import GaitVersionToolError
from .gait_version_tool_pop_up import PopUpWindow
from .parametric_pop_up import ParametricPopUpWindow
from .same_versions_pop_up import SameVersionsPopUpWindow

DEFAULT_AMOUNT_OF_AVAILABLE_SUBGAITS = 3
PARAMETRIC_GAIT_PREFIX = "_pg_"
FOUR_PARAMETRIC_GAIT_PREFIX = "_fpg_"


class LogLevel(Enum):
    INFO = "#000000"
    SUCCESS = "#009100"
    ERROR = "#FF0000"
    WARNING = "#b27300"


class GaitVersionToolView(QWidget):
    def __init__(self, ui_file, controller):
        """Base class to load and use the gait_selection.ui.

        :param ui_file:
            Path to the .ui file to load with the widget
        :param controller:
            A reference to the controller corresponding to the rqt gait selection functionality
        """
        super(GaitVersionToolView, self).__init__(flags=Qt.Window)

        self._controller = controller
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self)

        self._is_refresh_active = True
        self._is_update_active = True

        # Search for children in GUI
        self._subgaits = self.SubgaitsFrame
        self._gait_menu = self.GaitMenu
        self._logger = self.Log

        self._subgait_labels = []
        self._subgait_menus = []

        for index in range(DEFAULT_AMOUNT_OF_AVAILABLE_SUBGAITS):
            self._subgait_labels.append(
                getattr(self, "SubgaitLabel_{nr}".format(nr=index))
            )
            self._subgait_menus.append(
                getattr(self, "SubgaitMenu_{nr}".format(nr=index))
            )

        # bind functions to callbacks of buttons and menus
        self.Refresh.pressed.connect(lambda: self._refresh())
        self.Apply.pressed.connect(lambda: [self._apply(), self._refresh()])
        self.SelectSameVersions.pressed.connect(self._select_same_versions)
        self.SaveDefault.pressed.connect(
            lambda: [self._apply(), self._save_default(), self._refresh()]
        )

        self.ClearLogger.clicked.connect(lambda: self._logger.clear())
        self.SeeAllVersions.clicked.connect(lambda: self._show_version_map_pop_up())
        self.LoadSubgaits.clicked.connect(lambda: self.update_version_menus())

        self._gait_menu.currentIndexChanged.connect(lambda: self.update_version_menus())
        for subgait_menu in self._subgait_menus:
            subgait_menu.currentIndexChanged.connect(lambda: self.update_version())

        # loaded gaits from gait selection node
        self.available_gaits = {}
        self.version_map = {}

        # pop up windows
        self._version_map_pop_up = PopUpWindow(self)
        self._parametric_pop_up = ParametricPopUpWindow(
            self, ui_file.replace("gait_selection.ui", "parametric_pop_up.ui")
        )
        self._same_versions_pop_up = SameVersionsPopUpWindow(
            self, ui_file.replace("gait_selection.ui", "same_versions_pop_up.ui")
        )

        # populate gait menu for the first time
        self._refresh()

    # gait and subgait related layout functions
    def add_subgait_menus(self, amount_of_new_subgait_menus):
        """Add subgait labels and dropdown menu's in case a gait has more subgaits then available menu's.

        :param amount_of_new_subgait_menus: the amount of new subgait labels and menu's
        """
        for _ in range(amount_of_new_subgait_menus):

            new_subgait_label = QLabel(self)
            new_subgait_label.setFont(self._subgait_labels[0].font())
            new_subgait_label.setAlignment(self._subgait_labels[0].alignment())

            new_subgait_menu = QComboBox(self)
            new_subgait_menu.setFont(self._subgait_menus[0].font())
            new_subgait_menu.currentIndexChanged.connect(lambda: self.update_version())

            self._subgait_labels.append(new_subgait_label)
            self._subgait_menus.append(new_subgait_menu)

            self._subgaits.layout().addRow(new_subgait_label, new_subgait_menu)

    @staticmethod
    def sort_versions(versions):
        def version_sorter(version):
            """Used in the sort function to sort based on the last 2 digits of the
            version name. If there is no version number, 0 is returned to have
            it at the top of the list.

            :param version: str of the version
            """
            try:
                length = len(version)
                version_number = "".join(
                    char for char in version[length - 2 : length] if char.isdigit()
                )
                if version_number != "":
                    return int(version_number)
                else:
                    return 0
            except ValueError:
                return 0

        return sorted(versions, key=version_sorter)

    def update_version_menus(self):
        """When a gait is selected set the subgait labels and populate the subgait menus with the available versions."""

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
            versions = self.sort_versions(versions)

            subgait_label.setText(subgait_name)
            subgait_menu.addItems(versions)
            subgait_menu.addItem("parametric")

            try:
                current_version = self.version_map[gait_name][subgait_name]
                current_version_index = versions.index(current_version)
                subgait_menu.setCurrentIndex(current_version_index)

            except ValueError:
                if current_version.startswith(PARAMETRIC_GAIT_PREFIX):
                    subgait_menu.addItem(current_version)
                    subgait_menu.setCurrentIndex(subgait_menu.count() - 1)
                else:
                    self._log(
                        "Default version of {gn} {sgn} does not exist in loaded gaits.".format(
                            gn=gait_name, sgn=subgait_name
                        ),
                        LogLevel.ERROR,
                    )
            except KeyError:
                self._log(
                    "{gn} has no default version for {sgn}.".format(
                        gn=gait_name, sgn=subgait_name
                    ),
                    LogLevel.ERROR,
                )

            latest_used_index = index + 1

        for unused_index in range(latest_used_index, len(self._subgait_labels)):
            self._subgait_labels[unused_index].hide()
            self._subgait_menus[unused_index].hide()

        self._is_update_active = False

    def update_version(self):
        """Update the subgait labels with a specific color to represent a change in version."""
        if self._is_update_active or self._is_refresh_active:
            return

        gait_name = self._gait_menu.currentText()
        for subgait_label, subgait_menu in zip(
            self._subgait_labels, self._subgait_menus
        ):
            subgait_name = subgait_label.text()
            if subgait_name != "Unused":
                try:
                    if str(subgait_menu.currentText()) == "parametric":
                        versions = self.available_gaits[gait_name][subgait_name]
                        if self._show_parametric_pop_up(versions):
                            if self._parametric_pop_up.four_subgait_interpolation:
                                new_version = self.get_four_parametric_version()
                            else:
                                new_version = self.get_parametric_version()
                            subgait_label.setStyleSheet(
                                f"color:{LogLevel.WARNING.value}"
                            )
                            subgait_menu.addItem(new_version)
                            subgait_menu.setCurrentIndex(subgait_menu.count() - 1)
                        else:
                            # parametric pop up window unsuccessful stopped, reset version to default
                            current_version_index = versions.index(
                                self.version_map[gait_name][subgait_name]
                            )
                            subgait_menu.setCurrentIndex(
                                max(current_version_index - 1, 0)
                            )
                    if str(self.version_map[gait_name][subgait_name]) != str(
                        subgait_menu.currentText()
                    ):
                        subgait_label.setStyleSheet(f"color:{LogLevel.WARNING.value}")
                    else:
                        subgait_label.setStyleSheet(f"color:{LogLevel.INFO.value}")
                except KeyError:
                    pass

    def _refresh(self):
        """Request the gait map from the gait selection node and display the available gaits in the gait menu."""
        self._is_refresh_active = True
        self._clear_gui()

        try:
            self.available_gaits = self._controller.get_directory_structure()
            self.version_map = self._controller.get_version_map()
            self._gait_menu.addItems(sorted(self.available_gaits.keys()))

            self._log("Directory data refreshed", LogLevel.SUCCESS)
        except GaitVersionToolError as e:
            self._log(str(e), LogLevel.ERROR)
        finally:
            self._is_refresh_active = False
            self.update_version_menus()

    # logger
    def _log(self, msg, level=LogLevel.INFO):
        """Use the logger window in the GUI to display data.

        :param msg:
            The message to display in the widget
        :param color_tag:
            The tag which represents the color of the text in the screen (info, warning, error)
        """
        if level in [LogLevel.SUCCESS, LogLevel.INFO]:
            self._controller._node.get_logger().info(msg)
        elif level == LogLevel.WARNING:
            self._controller._node.get_logger().warn(msg)
        elif level == LogLevel.ERROR:
            self._controller._node.get_logger().error(msg)
        else:
            self._controller._node.get_logger().error(
                f"Unknown log level specified for message: {msg}",
            )
            return
        self._logger.appendHtml(
            f'<p style="color:{level.value}; white-space: pre-wrap;">{msg}</p>'
        )
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
            subgait_label.setStyleSheet("color:#000000")
            subgait_label.setText("Unused")

        if clear_gait_menu:
            self._gait_menu.clear()

    def _save_default(self):
        """Save the currently selected subgait versions as default values."""
        try:
            success, msg = self._controller.set_default_versions()
            if success:
                self._log(msg if msg else "Set new default versions", LogLevel.SUCCESS)
            else:
                self._log(
                    msg if msg else "Failed to set default versions", LogLevel.ERROR
                )
        except GaitVersionToolError as e:
            self._log(str(e), LogLevel.ERRROR)

    def _apply(self):
        """Apply newly selected subgait versions to the gait selection node."""
        gait_name = self._gait_menu.currentText()
        subgait_names = []
        versions = []

        for subgait_label, subgait_menu in zip(
            self._subgait_labels, self._subgait_menus
        ):
            if subgait_label.text() != "Unused":
                subgait_names.append(subgait_label.text())
                versions.append(subgait_menu.currentText())

        try:
            success, msg = self._controller.set_gait_version(
                gait_name, subgait_names, versions
            )
            if success:
                if not msg:
                    msg = f"Version change applied for {gait_name}: "

                    for index, subgait in enumerate(subgait_names):
                        msg += f"\n    - {subgait}:    {versions[index]}"
                self._log(msg, LogLevel.SUCCESS)
            else:
                self._log(
                    msg if msg else "Version change applied failed", LogLevel.ERROR
                )
        except GaitVersionToolError as e:
            self._log(str(e), LogLevel.ERROR)

    def _select_same_versions(self):
        """Select same versions of subgaits using common pre- and postfixes.

        For every version of every subgait:
            - Find the first match with the specified prefix and postfix
            - Set the subgait menu to that version
        """
        if not self._same_versions_pop_up.show_pop_up(self._gait_menu.currentText()):
            return

        prefix = self._same_versions_pop_up.prefix
        postfix = self._same_versions_pop_up.postfix
        if prefix == "" and postfix == "":
            return

        if prefix == "":
            prefix = ".*"
        if postfix == "":
            postfix = ".*"

        gait_name = self._gait_menu.currentText()
        if gait_name not in self.version_map:
            return

        subgaits = self.available_gaits[gait_name]
        selected_versions = {}
        for subgait, versions in subgaits.items():

            subgait_no_underscores = subgait.replace("_", "")
            regex_string = f"{prefix}(_?{gait_name})?_({subgait}|{subgait_no_underscores})_{postfix}"
            pattern = re.compile(regex_string)

            version_is_found = False
            for version_name in versions:
                match = pattern.match(version_name)
                if match is not None:
                    selected_versions[subgait] = version_name
                    version_is_found = True

            if not version_is_found:
                self._log(
                    f"Unable to find a matching version for subgait {subgait}",
                    LogLevel.WARNING,
                )

        for subgait_label, subgait_menu in zip(
            self._subgait_labels, self._subgait_menus
        ):
            subgait = subgait_label.text()
            if subgait in selected_versions:
                version_index = self.sort_versions(subgaits[subgait]).index(
                    selected_versions[subgait]
                )
                subgait_menu.setCurrentIndex(version_index)

    def _show_version_map_pop_up(self):
        """Use a pop up window to display all the gait, subgaits and currently used versions."""
        try:
            version_map = self._controller.get_version_map()
        except GaitVersionToolError as e:
            self._log(str(e), LogLevel.ERROR)
            return

        version_map_string = ""
        for gait_name in sorted(version_map.keys()):
            version_map_string += "{gait} \n".format(gait=gait_name)
            for subgait_name, version in version_map[gait_name].items():
                version_map_string += "\t{sb:<30} \t {vs} \n".format(
                    sb=subgait_name, vs=version
                )
            version_map_string += "\n"

        self._version_map_pop_up.show_message(version_map_string)

    def _show_parametric_pop_up(self, versions):
        """Use a pop up window to get the base version, other version and
        parameter for a parametric subgait."""
        return self._parametric_pop_up.show_pop_up(versions)

    def get_parametric_version(self):
        return "{0}{1}_({2})_({3})".format(
            PARAMETRIC_GAIT_PREFIX,
            self._parametric_pop_up.parameter,
            self._parametric_pop_up.base_version,
            self._parametric_pop_up.other_version,
        )

    def get_four_parametric_version(self):
        return "{0}{1}_{2}_({3})_({4})_({5})_({6})".format(
            FOUR_PARAMETRIC_GAIT_PREFIX,
            self._parametric_pop_up.first_parameter,
            self._parametric_pop_up.second_parameter,
            self._parametric_pop_up.first_version,
            self._parametric_pop_up.second_version,
            self._parametric_pop_up.third_version,
            self._parametric_pop_up.fourth_version,
        )
