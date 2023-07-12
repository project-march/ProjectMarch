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
        self._controller.accepted_cb = self._accepted_cb
        self._controller.finished_cb = self._finished_cb
        self._controller.rejected_cb = self._rejected_cb
        self._controller.current_gait_cb = self._current_gait_cb
        self.possible_gaits_future = None

        self._controller._node.create_subscription(
            msg_type=Bool,
            topic="/march/eeg/on_off",
            callback=self._eeg_cb,
            qos_profile=1,
        )

        self._always_enabled_buttons = []

        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self)

        self.refresh_button.clicked.connect(self._controller.update_possible_gaits)

        self.logger = logger
        self._layout_file = layout_file if layout_file != "" else DEFAULT_LAYOUT_FILE
        self._image_names = [
            file.name
            for file in Path(get_package_share_directory("march_rqt_input_device"), "resource", "img").glob("*.png")
        ]
        self._create_buttons()
        self._update_possible_gaits()

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

    def _accepted_cb(self) -> None:
        """Show the GaitInstructionResponse and update possible gaits."""
        self.status_label.setText("Gait accepted")
        self._update_possible_gaits()

    def _finished_cb(self) -> None:
        """Show the GaitInstructionResponse and update possible gaits."""
        self.status_label.setText("Gait finished")
        self.gait_label.setText("")
        self._update_possible_gaits()

    def _rejected_cb(self) -> None:
        """Show the GaitInstructionResponse and update possible gaits."""
        self.status_label.setText("Gait rejected")
        self.gait_label.setText("")
        self._update_possible_gaits()

    def _current_gait_cb(self, gait_name: str) -> None:
        """Show the current gait and update possible gaits."""
        self.gait_label.setText(gait_name)

    def _eeg_cb(self, data) -> None:
        """Update the possible gaits when eeg is turned on or off."""
        self._update_possible_gaits()

    def _update_possible_gaits(self) -> None:
        """Updates the gaits based on the possible gaits according to the controller.

        First requests the controller to update the possible, then creates a timer to update the view if the possible
        gaits changed.
        """
        if self._controller.use_mpc:
            self.possible_gaits = self._controller.update_mpc_gaits()
            self._update_gait_buttons(self.possible_gaits)
        else:
            self._controller.update_possible_gaits()
            self.possible_gaits = []
            self._update_gait_buttons([])
            self._controller.gait_future.add_done_callback(self._update_possible_gaits_view)

    def _update_possible_gaits_view(self, future) -> None:
        """Update the buttons if the possible gaits have changed."""
        new_possible_gaits = future.result().gaits
        if set(self.possible_gaits) != set(new_possible_gaits):
            self._update_gait_buttons(new_possible_gaits)

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
                if name in self._always_enabled_buttons:
                    continue
                if name in self.possible_gaits:
                    button.setEnabled(True)
                else:
                    button.setEnabled(False)
        self.frame.setEnabled(True)
        self.frame.verticalScrollBar().setEnabled(True)

    def create_button(
        self,
        name: str,
        mpc_command: bool = False,
        gait_type: Optional[int] = None,
        callback: Optional[Union[str, Callable]] = None,
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
                qt_button.clicked.connect(callback)
            else:
                qt_button.clicked.connect(getattr(self._controller, callback))
        if gait_type is not None:
            qt_button.clicked.connect(lambda: self.publish_mpc_gait(gait_type, mpc_command, control_type))
        else:
            qt_button.clicked.connect(lambda: self._controller.publish_gait(name, control_type))

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

    def publish_mpc_gait(self, gait_type: int, mpc_command: bool, control_type: str):
        """Publish gait to state_machine."""
        self._controller._node.get_logger().warn("\n\n MPC Gaits are selected! \n")
        self._controller.use_mpc = mpc_command
        self._controller.publish_mpc_gait(gait_type, control_type)
        if gait_type == 1:
            self._controller.publish_sm_to_unknown()
            self._controller.publish_gait("home_stand", "position")
        self._update_possible_gaits()


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
