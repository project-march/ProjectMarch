import json
import os
from typing import List, Callable, Tuple, Optional, Union

from pathlib import Path

from .input_device_controller import InputDeviceController
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QSize
from python_qt_binding.QtWidgets import QGridLayout
from python_qt_binding.QtWidgets import QPushButton
from python_qt_binding.QtWidgets import QWidget
from ament_index_python.packages import get_package_share_directory
from .image_button import ImageButton


class InputDeviceView(QWidget):
    """
    The View of the input device, inialized based on a ui file and a controller.
    """

    def __init__(
        self, ui_file: str, layout_file: str, controller: InputDeviceController, logger
    ):
        """
        Initializes the view with a UI file and controller.

        :type ui_file: str
        :param ui_file: path to a Qt UI file
        :type controller: InputDeviceController
        :param controller: input device controller for sending ROS messages
        """
        super(InputDeviceView, self).__init__()
        self._controller = controller
        self._controller.accepted_cb = self._accepted_cb
        self._controller.finished_cb = self._finished_cb
        self._controller.rejected_cb = self._rejected_cb
        self._controller.current_gait_cb = self._current_gait_cb
        self.possible_gaits_future = None

        self._always_enabled_buttons = []

        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self)

        self.refresh_button.clicked.connect(self._controller.update_possible_gaits)

        self.logger = logger
        self._layout_file = layout_file
        self._image_names = [
            file.name
            for file in Path(
                get_package_share_directory("march_rqt_input_device"), "resource", "img"
            ).glob("*.png")
        ]
        self._create_buttons()
        self._update_possible_gaits()

    def _create_buttons(self) -> None:
        """
        Creates all the buttons, new buttons should be added here.
        """
        json_content = json.loads(open(self._layout_file).read())

        button_layout = [
            [self.create_button(**button_dict) for button_dict in row]
            for row in json_content
        ]

        # Create the qt_layout from the button layout.
        qt_layout = self.create_layout(button_layout)

        # Apply the qt_layout to the top level widget.
        self.content.setLayout(qt_layout)

        # Make the frame as tight as possible with spacing between the buttons.
        qt_layout.setSpacing(15)
        self.content.adjustSize()

    def _accepted_cb(self) -> None:
        """
        Show the GaitInstructionResponse and update possible gaits
        """
        self.status_label.setText("Gait accepted")
        self._update_possible_gaits()

    def _finished_cb(self) -> None:
        """
        Show the GaitInstructionResponse and update possible gaits
        """
        self.status_label.setText("Gait finished")
        self.gait_label.setText("")
        self._update_possible_gaits()

    def _rejected_cb(self) -> None:
        """
        Show the GaitInstructionResponse and update possible gaits
        """
        self.status_label.setText("Gait rejected")
        self.gait_label.setText("")
        self._update_possible_gaits()

    def _current_gait_cb(self, gait_name: str) -> None:
        """
        Show the current gait and update possible gaits
        """
        self.gait_label.setText(gait_name)

    def _update_possible_gaits(self) -> None:
        """
        First requests the controller to update the possible, then create a timer to update the view if the possible
        gaits changed.
        """
        self._controller.update_possible_gaits()
        self.possible_gaits = []
        self._update_gait_buttons([])
        self._controller.gait_future.add_done_callback(self._update_possible_gaits_view)

    def _update_possible_gaits_view(self, future) -> None:
        """
        Update the buttons if the possible gaits have changed.
        """
        new_possible_gaits = future.result().gaits
        if set(self.possible_gaits) != set(new_possible_gaits):
            self._update_gait_buttons(new_possible_gaits)

    def _update_gait_buttons(self, possible_gaits: List[str]) -> None:
        """
        Update which buttons are available to the given possible gaits list
        @param possible_gaits: The gaits that can be executed
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
        callback: Optional[Union[str, Callable]] = None,
        image_path: Optional[str] = None,
        size: Tuple[int, int] = (128, 160),
        always_enabled: bool = False,
    ):
        """Create a push button which can be pressed to execute a gait instruction.

        :param name:
            Name of the button
        :param callback:
            The callback to attach to the button when pressed
        :param image_path:
            The name of the image file
        :param size:
            Default size of the button
        :param always_enabled:
            Never disable the button if it's not in possible gaits

        :return:
            A QPushButton object which contains the passed arguments
        """
        if image_path is not None:
            qt_button = ImageButton(get_image_path(image_path))
        elif name + ".png" in self._image_names:
            qt_button = ImageButton(get_image_path(name))
        else:
            qt_button = QPushButton()

            text = check_string(name)
            qt_button.setText(text)
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
        else:
            qt_button.clicked.connect(lambda: self._controller.publish_gait(name))

        return qt_button

    @staticmethod
    def create_layout(layout_list):
        """Create a button layout with a given list of buttons.

        :param layout_list:
            A list which contains multiple list which represent a row with given items

        :return:
            A populated QGridLayout object which contains the passed input buttons
        """
        qt_button_layout = QGridLayout()

        for row in range(len(layout_list)):
            for column in range(len(layout_list[row])):
                user_input_object = layout_list[row][column]
                qt_button_layout.addWidget(user_input_object, row, column, 1, 1)

        return qt_button_layout


def get_image_path(image_path: str) -> str:
    """Create an absolute image path to an image.

    If the image_path is already absolute, just return it.
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

    :type text: str
    :param text: The text to split

    :return New string which contains newlines
    """
    words = text.replace("_", " ").split(" ")
    new_string = words[0]
    for index, word in enumerate(words[1:], 1):
        new_string = (
            new_string + "\n" + word if index % 3 == 0 else new_string + " " + word
        )
    return new_string
