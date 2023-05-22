"""Author: MIV; MV; MVI."""
import json
import os
from typing import List, Callable, Tuple, Optional, Union

from pathlib import Path

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QIcon, QPixmap
from PyQt5.QtWidgets import QToolButton

from .input_device_controller import InputDeviceController
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QSize
from python_qt_binding.QtWidgets import QGridLayout
from python_qt_binding.QtWidgets import QWidget
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Bool

DEFAULT_LAYOUT_FILE = os.path.join(get_package_share_directory("march_rqt_input_device"), "config", "training.json")
MAX_CHARACTERS_PER_LINE_BUTTON = 17


class InputDeviceView(QWidget):
    """The View of the input device, initialized based on an ui file and a controller.

    Args:
        ui_file (str): Path to a Qt UI file.
        layout_file (str): The path to the layout.json file. If the file does not exist is uses the DEFAULT_LAYOUT_FILE.
            See "package[`march_rqt_input_device`]/config/training.json" file for an example layout file.
        controller (InputDeviceController): Input device controller for sending ROS messages.
    """

    def __init__(self, ui_file: str, layout_file: str, controller: InputDeviceController, logger):
        super(InputDeviceView, self).__init__()
        self._controller = controller

        self._controller.node.create_subscription(
            msg_type=Bool,
            topic="/march/eeg/on_off",
            callback=self._eeg_cb,
            qos_profile=1,
        )

        self._always_enabled_buttons = []

        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self)
        self.logger = logger
        self._layout_file = layout_file if layout_file != "" else DEFAULT_LAYOUT_FILE
        self._image_names = [
            file.name
            for file in Path(get_package_share_directory("march_rqt_input_device"), "resource", "img").glob("*.png")
        ]
        self._create_buttons()
        self._update_possible_gaits()

    def _eeg_cb(self, data) -> None:
        """Update the possible gaits when eeg is turned on or off."""
        self._update_possible_gaits()
        self._controller.update_eeg_on_off(data)

    def publish_gait(self, gait_type: int):
        """Publish gait to state_machine."""
        self._controller.publish_gait(gait_type)
        self._update_possible_gaits()

    def _update_possible_gaits(self) -> None:
        """Updates the gaits based on the possible gaits according to the controller.

        First requests the controller to update the possible, then creates a timer to update the view if the possible
        gaits changed.
        """
        self.possible_gaits = self._controller.update_possible_gaits()
        self._controller.get_node().get_logger().debug("possible gaits: " + str(self.possible_gaits))
        self._update_gait_buttons(self.possible_gaits)

    def _update_gait_buttons(self, possible_gaits: List[str]) -> None:
        """Update which buttons are available to the given possible gaits list.

        Args:
            possible_gaits (List[str]): List of gaits names that can be executed.
        """
        self.frame.setEnabled(False)
        self.frame.verticalScrollBar().setEnabled(False)
        self.possible_gaits = possible_gaits

        layout = self.content.layout()
        if layout:
            for i in range(layout.count()):
                button = layout.itemAt(i).widget()
                name = button.objectName()
                if len(possible_gaits) == 0:
                    button.setEnabled(False)
                    continue
                if name in self._always_enabled_buttons:
                    continue
                if name in self.possible_gaits:
                    button.setEnabled(True)
                else:
                    button.setEnabled(False)
        self.frame.setEnabled(True)
        self.frame.verticalScrollBar().setEnabled(True)

    def _create_buttons(self) -> None:
        """Creates all the buttons, new buttons should be added here."""
        with open(self._layout_file) as file:
            json_content = json.loads(file.read())

        button_layout = [[self.create_button(**button_dict) for button_dict in row] for row in json_content]

        # Create the qt_layout from the button layout.
        qt_layout = self.create_layout(button_layout)

        # Apply the qt_layout to the top level widget.
        self.content.setLayout(qt_layout)

        # Make the frame as tight as possible with spacing between the buttons.
        qt_layout.setSpacing(10)
        self.content.adjustSize()

    def create_button(
        self,
        name: str,
        gait_type: Optional[int] = None,
        callback: Optional[Union[str, Callable]] = None,
        control_type: Optional[str] = None,
        image_path: Optional[str] = None,
        size: Tuple[int, int] = (125, 140),
        always_enabled: bool = False,
    ):
        """Create a push button which can be pressed to execute a gait instruction.

        :param name: name of button;
        :param gait_type: int of gait type started by the button;
        :param control_type (str, Optional): the name of the control type. Options are "fuzzy", "position", "torque". Default is None
        :param callback: callback linked to the button;
        :param image_path: path to the button image;
        :param size: size of the button;
        :param always_enabled: if the button should always be visible.
        :return:
        """
        qt_button = QToolButton()
        qt_button.setToolButtonStyle(Qt.ToolButtonTextUnderIcon)
        qt_button.setStyleSheet("QToolButton {background-color: white; font-size: 13px; font: 'Montserat'}")
        qt_button.setIconSize(QSize(90, 90))
        qt_button.setText(check_string(name) + "\n" + check_string(control_type))
        if image_path is not None:
            qt_button.setIcon(QIcon(QPixmap(get_image_path(image_path))))
        elif name + ".png" in self._image_names:
            qt_button.setIcon(QIcon(QPixmap(get_image_path(name))))
        qt_button.setObjectName(name)

        if always_enabled:
            self._always_enabled_buttons.append(name)
            qt_button.setEnabled(True)

        qt_button.setMinimumSize(QSize(*size))
        qt_button.setMaximumSize(QSize(*size))

        if callback is not None:
            if callable(callback):
                self._controller.get_node().get_logger().info(name + "'s callback " + callback + "  is callable")
                qt_button.clicked.connect(callback)
            else:
                self._controller.get_node().get_logger().info(name + "'s callback " + callback + "  is NOT callable")
                qt_button.clicked.connect(getattr(self._controller, callback))
        else:
            self._controller.get_node().get_logger().info(name + "'s callback is none")
            qt_button.clicked.connect(lambda: self.publish_gait(gait_type, control_type))

        return qt_button

    @staticmethod
    def create_layout(layout_list):
        """Create a button layout with a given list of buttons.

        Args:
            layout_list: A list which contains multiple list which represent a row with given items.

        Returns:
            QGridLayout. A populated QGridLayout object which contains the passed input buttons.
        """
        qt_button_layout = QGridLayout()

        for row in range(len(layout_list)):
            for column in range(len(layout_list[row])):
                user_input_object = layout_list[row][column]
                qt_button_layout.addWidget(user_input_object, row, column, 1, 1)

        return qt_button_layout


def get_image_path(image_path: str) -> str:
    """Returns an absolute image path to an image.

    Returns:
        str. The image_path if it is already absolute,
            otherwise it makes an absolute image path.
    """
    if os.path.isabs(image_path):
        return image_path
    else:
        return os.path.join(
            get_package_share_directory("march_rqt_input_device"),
            "resource",
            "img",
            f"{image_path}.png",
        )


def check_string(text: str) -> str:
    """Split text into new lines on every third word.

    Args:
        text (str): The text to split.

    Returns:
         str. The string that has a new word on every third word.
    """
    if(text == None):
        return ""
    words = text.replace("_", " ").split(" ")
    new_string = words[0]
    characters_since_line_break = len(new_string)
    for _, word in enumerate(words[1:], 1):
        if characters_since_line_break + len(word) > MAX_CHARACTERS_PER_LINE_BUTTON:
            new_string = new_string + "\n" + word
            characters_since_line_break = len(word)
        else:
            new_string = new_string + " " + word
            characters_since_line_break = len(word) + characters_since_line_break + 1

    return new_string
