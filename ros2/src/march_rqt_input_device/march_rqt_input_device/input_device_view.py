"""Author: MIV; MV; MVI, MIX"""
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
from march_rqt_input_device.exo_mode import ExoMode
import time

DEFAULT_LAYOUT_FILE = os.path.join("src/march_mode_machine/", "generate", "modes.json")
MAX_CHARACTERS_PER_LINE_BUTTON = 17


class InputDeviceView(QWidget):
    def __init__(self, ui_file: str, controller: InputDeviceController, layout_file: str = None):
        super(InputDeviceView, self).__init__()

        loadUi(ui_file, self)

        self._button_layout = None
        self._current_possible_modes = set()


        self._layout_file = layout_file
        self._controller = controller

        self._create_buttons()

    def _create_buttons(self) -> None:
        """Creates all the buttons, new buttons should be added here."""
        with open(self._layout_file) as file:
            json_content = json.loads(file.read())

        self._button_layout = [[self.create_button(**button_dict) for button_dict in row if button_dict['name'] not in ['BootUp', 'Error']] for row in json_content]

        # Create the qt_layout from the button layout.
        qt_layout = self.create_layout(self._button_layout)

        # Apply the qt_layout to the top level widget.
        self.content.setLayout(qt_layout)

        # Make the frame as tight as possible with spacing between the buttons.
        qt_layout.setSpacing(10)
        self.content.adjustSize()

    def create_button(
        self,
        name: str,
        callback: Optional[Union[str, Callable]] = None,
        ExoMode: Optional[int] = None,
        control_type: Optional[str] = None,
        image_path: Optional[str] = None,
        size: Tuple[int, int] = (125, 140),
        always_enabled: bool = False,
    ):
        """Create a push button which can be pressed to execute a gait instruction.

        Args:
            name (str): Name of the button.
            callback (Union[str, Callable], Optional): The callback to attach to the button when pressed.
            image_path (str, Optional): The name of the image file. Default is `None`.
            size ((int,int)): Size of the button in pixels in format (width, height). Default is (w=125px, h=140px).
            always_enabled (bool): Whether the button can be disabled. Default is False.
                `True` if the button should never be disabled
                `False` if the button should be disabled if it's not in possible gaits.

        Returns:
            QPushButton. The QPushButton contains the passed arguments and the same style.
        """
        qt_button = QToolButton()
        qt_button.setToolButtonStyle(Qt.ToolButtonTextUnderIcon)
        qt_button.setStyleSheet("QToolButton {background-color: lightgrey; font-size: 13px; font: 'Times New Roman'}")
        qt_button.setIconSize(QSize(90, 90))
        qt_button.setText(check_string(name) + "\n" + check_string(control_type))
        qt_button.setObjectName(name)

        if qt_button.objectName() == "Stand":
            qt_button.setEnabled(True)
        else:
            qt_button.setEnabled(False)

        qt_button.setMinimumSize(QSize(*size))
        qt_button.setMaximumSize(QSize(*size))

        if ExoMode is not None:
            # Check if a method with the name specified by `callback`` exists in the controller class and if it is callable/a function.
            qt_button.clicked.connect(lambda: self._controller.publish_mode(ExoMode))
        else:
            # If no exomode is defined, print a warning.
            print(f"Exomode is not defined.")


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
    
    def update_possible_modes(self) -> None:
        self._controller._available_modes_future.add_done_callback(self._controller.store_available_modes)

    def update_buttons(self, available_modes) -> None:
        available_mode_names = [ExoMode(mode).name for mode in available_modes]
        for button_row in self._button_layout:
            for button in button_row:
                if button.objectName() in available_mode_names:
                    button.setEnabled(True)
                else:
                    button.setEnabled(False)

def check_string(text: str) -> str:
    """Split text into new lines on every third word.

    Args:
        text (str): The text to split.

    Returns:
         str. The string that has a new word on every third word.
    """
    if text is None:
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
