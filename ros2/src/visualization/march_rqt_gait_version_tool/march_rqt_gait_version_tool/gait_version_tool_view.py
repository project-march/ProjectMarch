"""Author: MV; MVI."""
from enum import Enum
from typing import List, Dict

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QComboBox, QLabel, QWidget
from python_qt_binding import loadUi

from .gait_version_tool_errors import GaitVersionToolError
from .gait_version_tool_pop_up import PopUpWindow
from .parametric_pop_up import ParametricPopUpWindow
from .parametric_same_versions_pop_up import ParametricSameVersionsPopUpWindow
from .same_versions_pop_up import SameVersionsPopUpWindow
from .subgait_version_select import select_same_subgait_versions

DEFAULT_AMOUNT_OF_AVAILABLE_SUBGAITS = 3
PARAMETRIC_GAIT_PREFIX = "_pg_"
FOUR_PARAMETRIC_GAIT_PREFIX = "_fpg_"


class LogLevel(Enum):
    """Enum for log level color codes."""

    INFO = "#000000"
    SUCCESS = "#009100"
    ERROR = "#FF0000"
    WARNING = "#b27300"


class GaitVersionToolView(QWidget):
    """Base class to load and use the `gait_selection.ui`.

    Args:
        ui_file: Path to the .ui file to load with the widget.
        controller: A reference to the controller corresponding to the rqt gait selection functionality.
    """

    def __init__(self, ui_file, controller):
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
            self._subgait_labels.append(getattr(self, "SubgaitLabel_{nr}".format(nr=index)))
            self._subgait_menus.append(getattr(self, "SubgaitMenu_{nr}".format(nr=index)))

        # bind functions to callbacks of buttons and menus
        self.Refresh.pressed.connect(lambda: self._refresh())
        self.Apply.pressed.connect(lambda: [self._apply(), self._refresh()])
        self.SelectSameVersions.pressed.connect(self._select_same_versions)
        self.ParametricSameVersions.pressed.connect(self._parameterize_same_versions)
        self.SaveDefault.pressed.connect(lambda: [self._apply(), self._save_default(), self._refresh()])

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
        self._parametric_same_versions_pop_up = ParametricSameVersionsPopUpWindow(
            self,
            ui_file.replace("gait_selection.ui", "parametric_same_versions_pop_up.ui"),
        )

        # populate gait menu for the first time
        self._refresh()

    # gait and subgait related layout functions
    def add_subgait_menus(self, amount_of_new_subgait_menus):
        """Add subgait labels and dropdown menu's in case a gait has more subgaits then available menu's.

        Args:
             amount_of_new_subgait_menus: the amount of new subgait labels and menu's
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
    def sort_versions(versions: List[str]) -> List[str]:
        """Sorts the versions on the last 2 digits of the version name.

        Args:
            versions (List[str]): A list of versions to sort, where each version should end with 2 digits in the name,
                or it is bumped to the beginning of the sorted list.

        Returns:
            List[str]: A sorted list of the versions.
        """

        def version_sorter(version):
            """Comparator function used to evaluate the sorted position of a version string.

            This method is passed as an evaluate function for the default python `sorted` method.
            This function sorts based on the last 2 digits of the version name, if there is no version number,
            0 is returned to have it at the top of the list.

            Args:
                 version: str of the version.

            Returns:
                int: Position value for sort functions.
            """
            try:
                length = len(version)
                version_number = "".join(char for char in version[length - 2 : length] if char.isdigit())
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

        gait_name = self.current_gait
        subgaits = self.available_gaits[self.current_gait]

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
                    "{gn} has no default version for {sgn}.".format(gn=gait_name, sgn=subgait_name),
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

        gait_name = self.current_gait
        for subgait_label, subgait_menu in zip(self._subgait_labels, self._subgait_menus):
            subgait_name = subgait_label.text()
            if subgait_name != "Unused":
                try:
                    if str(subgait_menu.currentText()) == "parametric":
                        versions = self.available_gaits[gait_name][subgait_name]
                        if self._show_parametric_pop_up(versions):
                            new_version = self.get_parametric_version(
                                self._parametric_pop_up.parameters,
                                self._parametric_pop_up.selected_versions,
                            )
                            subgait_label.setStyleSheet(f"color:{LogLevel.WARNING.value}")
                            subgait_menu.addItem(new_version)
                            subgait_menu.setCurrentIndex(subgait_menu.count() - 1)
                        else:
                            # parametric pop up window unsuccessful stopped, reset version to default
                            current_version_index = versions.index(self.version_map[gait_name][subgait_name])
                            subgait_menu.setCurrentIndex(max(current_version_index - 1, 0))
                    if str(self.version_map[gait_name][subgait_name]) != str(subgait_menu.currentText()):
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

        Args:
            msg (str): The message to display in the widget.
            level (LogLevel) : The tag which represents the color of the text in the screen (info, warning, error).
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
        self._logger.appendHtml(f'<p style="color:{level.value}; white-space: pre-wrap;">{msg}</p>')
        scrollbar = self.Log.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    # button functions
    def _clear_gui(self, clear_gait_menu=False):
        """Clear the subgait menus and subgait labels and optionally the gait menu.

        Args:
            clear_gait_menu: Set this to true if the gait menu should also be cleared.
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
                self._log(msg if msg else "Failed to set default versions", LogLevel.ERROR)
        except GaitVersionToolError as e:
            self._log(str(e), LogLevel.ERRROR)

    def _apply(self):
        """Apply newly selected subgait versions to the gait selection node."""
        gait_name = self.current_gait
        subgait_names = []
        versions = []

        for subgait_label, subgait_menu in zip(self._subgait_labels, self._subgait_menus):
            if subgait_label.text() != "Unused":
                subgait_names.append(subgait_label.text())
                versions.append(subgait_menu.currentText())

        try:
            success, msg = self._controller.set_gait_version(gait_name, subgait_names, versions)
            if success:
                if not msg:
                    msg = f"Version change applied for {gait_name}: "

                    for index, subgait in enumerate(subgait_names):
                        msg += f"\n    - {subgait}:    {versions[index]}"
                self._log(msg, LogLevel.SUCCESS)
            else:
                self._log(msg if msg else "Version change applied failed", LogLevel.ERROR)
        except GaitVersionToolError as e:
            self._log(str(e), LogLevel.ERROR)

    def _select_same_versions(self):
        """Select same versions of subgaits using common pre- and postfixes.

        For every version of every subgait:
            - Find the first match with the specified prefix and postfix
            - Set the subgait menu to that version
        """
        gait_name = self.current_gait
        if not self._same_versions_pop_up.show_pop_up(gait_name):
            return

        prefix = self._same_versions_pop_up.prefix
        postfix = self._same_versions_pop_up.postfix
        subgaits = self.available_gaits[gait_name]

        selected_versions = select_same_subgait_versions(gait_name, subgaits, prefix, postfix)

        self._update_selected_subgaits(selected_versions)

    def _update_selected_subgaits(self, selected_versions: Dict[str, str]):
        """Set the subgait dropdown menus to the subgaits of the selected versions.

        Args:
             selected_versions (Dict[str, str]): Dictionary mapping subgait to selected version.
        """
        for subgait_label, subgait_menu in zip(self._subgait_labels, self._subgait_menus):
            subgait = subgait_label.text()
            if subgait in selected_versions:
                version_index = self.sort_versions(self.available_gaits[self.current_gait][subgait]).index(
                    selected_versions[subgait]
                )
                subgait_menu.setCurrentIndex(version_index)

    def _parameterize_same_versions(self):
        """Parameterize subgait that share a common pre- and postfix.

        This can be seen as a combination of the 'select_same_versions' pop-up and the 'parametric' pop-up.
        """
        if not self._parametric_same_versions_pop_up.show_pop_up(
            self.current_gait, self.available_gaits[self.current_gait]
        ):
            return

        parameters = self._parametric_same_versions_pop_up.parameters
        all_selected_versions = self._parametric_same_versions_pop_up.all_selected_versions

        # Convert list of subgait dictionaries to dictionary with a list for each subgait
        subgait_versions = {
            subgait: [selected_subgaits[subgait] for selected_subgaits in all_selected_versions]
            for subgait in self.available_gaits[self.current_gait]
        }

        for subgait_label, subgait_menu in zip(self._subgait_labels, self._subgait_menus):
            subgait = subgait_label.text()

            if subgait != "Unused":
                new_version = self.get_parametric_version(parameters, subgait_versions[subgait])

                subgait_label.setStyleSheet(f"color:{LogLevel.WARNING.value}")
                subgait_menu.addItem(new_version)
                subgait_menu.setCurrentIndex(subgait_menu.count() - 1)

    def _show_version_map_pop_up(self):
        """Use a pop-up window to display all the gait, subgaits and currently used versions."""
        try:
            version_map = self._controller.get_version_map()
        except GaitVersionToolError as e:
            self._log(str(e), LogLevel.ERROR)
            return

        version_map_string = ""
        for gait_name in sorted(version_map.keys()):
            version_map_string += "{gait} \n".format(gait=gait_name)
            for subgait_name, version in version_map[gait_name].items():
                version_map_string += "\t{sb:<30} \t {vs} \n".format(sb=subgait_name, vs=version)
            version_map_string += "\n"

        self._version_map_pop_up.show_message(version_map_string)

    def _show_parametric_pop_up(self, versions):
        """Use a pop-up window to get the base version, other version and parameter for a parametric subgait."""
        return self._parametric_pop_up.show_pop_up(versions)

    @staticmethod
    def get_parametric_version(parameters: List[float], subgait_versions: List[str]):
        """Create a parametric version name based on a list of subgait versions and a list of parameters.

        Args:
            parameters (List[float]): List of parameters, either one or two parameters.
            subgait_versions (List[str]): Subgait base, other, third, fourth versions.
        """
        if len(parameters) == 1 and len(subgait_versions) == 2:
            return "{0}{1}_({2})_({3})".format(
                PARAMETRIC_GAIT_PREFIX,
                parameters[0],
                subgait_versions[0],
                subgait_versions[1],
            )
        elif len(parameters) == 2 and len(subgait_versions) == 4:
            return "{0}{1}_{2}_({3})_({4})_({5})_({6})".format(
                FOUR_PARAMETRIC_GAIT_PREFIX,
                parameters[0],
                parameters[1],
                subgait_versions[0],
                subgait_versions[1],
                subgait_versions[2],
                subgait_versions[3],
            )
        else:
            raise Exception("Number of parameters and subgaits to parameterize do not match")

    @property
    def current_gait(self):
        """Returns the name of the current gait."""
        return self._gait_menu.currentText()
