import os

from python_qt_binding import loadUi
from python_qt_binding.QtCore import QSize
from python_qt_binding.QtWidgets import QGridLayout
from python_qt_binding.QtWidgets import QPushButton
from python_qt_binding.QtWidgets import QWidget
import rospkg

from .image_button import ImageButton


class InputDeviceView(QWidget):
    def __init__(self, ui_file, controller):
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

        self._always_enabled_buttons = []

        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self)

        self.refresh_button.clicked.connect(self._update_possible_gaits)

        self._create_buttons()
        self._update_possible_gaits()

    def _create_buttons(self):
        # Create buttons here
        rocker_switch_increment = \
            self.create_button('rocker_switch_up', image_path='/rocker_switch_up.png',
                               callback=lambda: self._controller.publish_increment_step_size(),
                               always_enabled=True)

        rocker_switch_decrement = \
            self.create_button('rocker_switch_down', image_path='/rocker_switch_down.png',
                               callback=lambda: self._controller.publish_decrement_step_size(),
                               always_enabled=True)

        home_sit = \
            self.create_button('home_sit', image_path='/home_sit.png',
                               callback=lambda: self._controller.publish_gait('home_sit'))

        home_stand = \
            self.create_button('home_stand', image_path='/home_stand.png',
                               callback=lambda: self._controller.publish_gait('home_stand'))

        gait_sit = \
            self.create_button('gait_sit', image_path='/gait_sit.png',
                               callback=lambda: self._controller.publish_gait('gait_sit'))

        gait_walk = \
            self.create_button('gait_walk', image_path='/gait_walk.png',
                               callback=lambda: self._controller.publish_gait('gait_walk'))

        gait_walk_small = \
            self.create_button('gait_walk_small', image_path='/gait_walk_small.png',
                               callback=lambda: self._controller.publish_gait('gait_walk_small'))

        gait_walk_large = \
            self.create_button('gait_walk_large', image_path='/gait_walk_large.png',
                               callback=lambda: self._controller.publish_gait('gait_walk_large'))

        gait_single_step_small = \
            self.create_button('gait_single_step_small', image_path='/gait_single_step_small.png',
                               callback=lambda: self._controller.publish_gait('gait_single_step_small'))

        gait_single_step_normal = \
            self.create_button('gait_single_step_normal', image_path='/gait_single_step_medium.png',
                               callback=lambda: self._controller.publish_gait('gait_single_step_normal'))

        gait_side_step_left = \
            self.create_button('gait_side_step_left', image_path='/gait_side_step_left.png',
                               callback=lambda: self._controller.publish_gait('gait_side_step_left'))

        gait_side_step_right = \
            self.create_button('gait_side_step_right', image_path='/gait_side_step_right.png',
                               callback=lambda: self._controller.publish_gait('gait_side_step_right'))

        gait_side_step_left_small = \
            self.create_button('gait_side_step_left_small',
                               callback=lambda: self._controller.publish_gait('gait_side_step_left_small'))

        gait_side_step_right_small = \
            self.create_button('gait_side_step_right_small',
                               callback=lambda: self._controller.publish_gait('gait_side_step_right_small'))

        gait_stand = \
            self.create_button('gait_stand', image_path='/gait_stand.png',
                               callback=lambda: self._controller.publish_gait('gait_stand'))

        gait_sofa_stand = \
            self.create_button('gait_sofa_stand', image_path='/gait_sofa_stand.png',
                               callback=lambda: self._controller.publish_gait('gait_sofa_stand'))

        gait_sofa_sit = \
            self.create_button('gait_sofa_sit', image_path='/gait_sofa_sit.png',
                               callback=lambda: self._controller.publish_gait('gait_sofa_sit'))

        gait_stairs_up = \
            self.create_button('gait_stairs_up', image_path='/gait_stairs_up.png',
                               callback=lambda: self._controller.publish_gait('gait_stairs_up'))

        gait_stairs_down = \
            self.create_button('gait_stairs_down', image_path='/gait_stairs_down.png',
                               callback=lambda: self._controller.publish_gait('gait_stairs_down'))

        gait_stairs_up_single_step = \
            self.create_button('gait_stairs_up_single_step', image_path='/gait_stairs_up_single_step.png',
                               callback=lambda: self._controller.publish_gait('gait_stairs_up_single_step'))

        gait_stairs_down_single_step = \
            self.create_button('gait_stairs_down_single_step', image_path='/gait_stairs_down_single_step.png',
                               callback=lambda: self._controller.publish_gait('gait_stairs_down_single_step'))

        gait_rough_terrain_high_step = \
            self.create_button('gait_rough_terrain_high_step', image_path='/gait_rough_terrain_high_step.png',
                               callback=lambda: self._controller.publish_gait('gait_rough_terrain_high_step'))

        gait_rough_terrain_middle_steps = \
            self.create_button('gait_rough_terrain_middle_steps',
                               callback=lambda: self._controller.publish_gait('gait_rough_terrain_middle_steps'))

        gait_rough_terrain_first_middle_step = \
            self.create_button('gait_rough_terrain_first_middle_step',
                               image_path='/gait_rough_terrain_first_middle_step.png',
                               callback=lambda: self._controller.publish_gait('gait_rough_terrain_first_middle_step'))

        gait_rough_terrain_second_middle_step = \
            self.create_button('gait_rough_terrain_second_middle_step',
                               image_path='/gait_rough_terrain_second_middle_step.png',
                               callback=lambda: self._controller.publish_gait('gait_rough_terrain_second_middle_step'))

        gait_rough_terrain_third_middle_step = \
            self.create_button('gait_rough_terrain_third_middle_step',
                               image_path='/gait_rough_terrain_third_middle_step.png',
                               callback=lambda: self._controller.publish_gait('gait_rough_terrain_third_middle_step'))

        gait_ramp_door_slope_up = \
            self.create_button('gait_ramp_door_slope_up', image_path='/gait_ramp_door_slope_up.png',
                               callback=lambda: self._controller.publish_gait('gait_ramp_door_slope_up'))

        gait_ramp_door_slope_down = \
            self.create_button('gait_ramp_door_slope_down', image_path='/gait_ramp_door_slope_down.png',
                               callback=lambda: self._controller.publish_gait('gait_ramp_door_slope_down'))

        gait_ramp_door_last_step = \
            self.create_button('gait_ramp_door_last_step', image_path='/gait_ramp_door_last_step.png',
                               callback=lambda: self._controller.publish_gait('gait_ramp_door_last_step'))

        gait_tilted_path_left_straight_start = \
            self.create_button('gait_tilted_path_left_straight_start',
                               image_path='/gait_tilted_path_left_straight_start.png',
                               callback=lambda: self._controller.publish_gait('gait_tilted_path_left_straight_start'))

        gait_tilted_path_left_single_step = \
            self.create_button('gait_tilted_path_left_single_step',
                               image_path='/gait_tilted_path_left_single_step.png',
                               callback=lambda: self._controller.publish_gait('gait_tilted_path_left_single_step'))

        gait_tilted_path_left_straight_end = \
            self.create_button('gait_tilted_path_left_straight_end',
                               image_path='/gait_tilted_path_left_straight_end.png',
                               callback=lambda: self._controller.publish_gait('gait_tilted_path_left_straight_end'))

        gait_tilted_path_left_flexed_knee_step = \
            self.create_button('gait_tilted_path_left_flexed_knee_step',
                               callback=lambda: self._controller.publish_gait('gait_tilted_path_left_flexed_knee_step'))

        gait_tilted_path_left_knee_bend = \
            self.create_button('gait_tilted_path_left_knee_bend',
                               callback=lambda: self.controller.publish_gait('gait_tilted_path_left_knee_bend'))

        gait_tilted_path_right_straight_start = \
            self.create_button('gait_tilted_path_right_straight_start',
                               image_path='/gait_tilted_path_right_straight_start.png',
                               callback=lambda: self._controller.publish_gait('gait_tilted_path_right_straight_start'))

        gait_tilted_path_right_single_step = \
            self.create_button('gait_tilted_path_right_single_step',
                               image_path='/gait_tilted_path_right_single_step.png',
                               callback=lambda: self._controller.publish_gait('gait_tilted_path_right_single_step'))

        gait_tilted_path_right_straight_end = \
            self.create_button('gait_tilted_path_right_straight_end',
                               image_path='/gait_tilted_path_right_straight_end.png',
                               callback=lambda: self._controller.publish_gait('gait_tilted_path_right_straight_end'))

        gait_tilted_path_right_flexed_knee_step = \
            self.create_button('gait_tilted_path_right_flexed_knee_step',
                               callback=lambda: self._controller.publish_gait(
                                   'gait_tilted_path_right_flexed_knee_step'))

        gait_tilted_path_right_knee_bend = \
            self.create_button('gait_tilted_path_right_knee_bend',
                               callback=lambda: self._controller.publish_gait('gait_tilted_path_right_knee_bend'))

        gait_tilted_path_first_start = \
            self.create_button('gait_tilted_path_first_start',
                               callback=lambda: self._controller.publish_gait('gait_tilted_path_first_start'))

        gait_tilted_path_second_start = \
            self.create_button('gait_tilted_path_second_start',
                               callback=lambda: self._controller.publish_gait('gait_tilted_path_second_start'))

        gait_tilted_path_first_end = \
            self.create_button('gait_tilted_path_first_end',
                               callback=lambda: self._controller.publish_gait('gait_tilted_path_first_end'))

        gait_tilted_path_second_end = \
            self.create_button('gait_tilted_path_second_end',
                               callback=lambda: self._controller.publish_gait('gait_tilted_path_second_end'))

        stop_button = self.create_button('gait_stop', image_path='/stop.png',
                                         callback=lambda: self._controller.publish_stop(),
                                         always_enabled=True)

        pause_button = self.create_button('gait_pause', image_path='/pause.png',
                                          callback=lambda: self._controller.publish_pause(),
                                          always_enabled=True)

        continue_button = self.create_button('gait_continue', image_path='/continue.png',
                                             callback=lambda: self._controller.publish_continue(),
                                             always_enabled=True)

        error_button = self.create_button('error', image_path='/error.png',
                                          callback=lambda: self._controller.publish_error(),
                                          always_enabled=True)

        sm_to_unknown_button = self.create_button('force unknown',
                                                  callback=lambda: self._controller.publish_sm_to_unknown(),
                                                  always_enabled=True)

        # The button layout.
        # Position in the array determines position on screen.
        march_button_layout = [

            [home_sit, home_stand, gait_walk, gait_walk_small, gait_walk_large, sm_to_unknown_button],

            [gait_sit, gait_stand, rocker_switch_increment, rocker_switch_decrement, stop_button, error_button],

            [gait_sofa_sit, gait_sofa_stand, gait_single_step_normal, gait_single_step_small, continue_button,
             pause_button],

            [gait_stairs_up, gait_stairs_down, gait_stairs_up_single_step, gait_stairs_down_single_step],

            [gait_side_step_left, gait_side_step_right, gait_side_step_left_small, gait_side_step_right_small],

            [gait_rough_terrain_high_step, gait_rough_terrain_middle_steps, gait_rough_terrain_first_middle_step,
             gait_rough_terrain_second_middle_step, gait_rough_terrain_third_middle_step],

            [gait_ramp_door_slope_up, gait_ramp_door_slope_down, gait_ramp_door_last_step],

            [gait_tilted_path_left_straight_start, gait_tilted_path_left_single_step,
             gait_tilted_path_left_straight_end, gait_tilted_path_left_flexed_knee_step,
             gait_tilted_path_left_knee_bend],

            [gait_tilted_path_right_straight_start, gait_tilted_path_right_single_step,
             gait_tilted_path_right_straight_end, gait_tilted_path_right_flexed_knee_step,
             gait_tilted_path_right_knee_bend],

            [gait_tilted_path_first_start, gait_tilted_path_second_start, gait_tilted_path_first_end,
             gait_tilted_path_second_end],
        ]

        # Create the qt_layout from the button layout.
        qt_layout = self.create_layout(march_button_layout)

        # Apply the qt_layout to the top level widget.
        self.content.setLayout(qt_layout)

        # Make the frame as tight as possible with spacing between the buttons.
        qt_layout.setSpacing(15)
        self.content.adjustSize()

    def _accepted_cb(self):
        self.status_label.setText('Gait accepted')
        self._update_possible_gaits()

    def _finished_cb(self):
        self.status_label.setText('Gait finished')
        self.gait_label.setText('')
        self._update_possible_gaits()

    def _rejected_cb(self):
        self.status_label.setText('Gait rejected')
        self.gait_label.setText('')
        self._update_possible_gaits()

    def _current_gait_cb(self, gait_name):
        self.gait_label.setText(gait_name)

    def _update_possible_gaits(self):
        self.frame.setEnabled(False)
        self.frame.verticalScrollBar().setEnabled(False)
        possible_gaits = self._controller.get_possible_gaits()
        layout = self.content.layout()
        if layout:
            for i in range(layout.count()):
                button = layout.itemAt(i).widget()
                name = button.objectName()
                if name in self._always_enabled_buttons:
                    continue

                if name in possible_gaits:
                    button.setEnabled(True)
                else:
                    button.setEnabled(False)
        self.frame.setEnabled(True)
        self.frame.verticalScrollBar().setEnabled(True)

    def create_button(self, name, callback=None, image_path=None, size=(128, 160), always_enabled=False):
        """Create a push button which the mock input device can register.

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
        if image_path is None:
            qt_button = QPushButton()

            text = check_string(name)
            qt_button.setText(text)
        else:
            qt_button = ImageButton(get_image_path(image_path))

        qt_button.setObjectName(name)

        if always_enabled:
            self._always_enabled_buttons.append(name)
            qt_button.setEnabled(True)

        qt_button.setMinimumSize(QSize(*size))
        qt_button.setMaximumSize(QSize(*size))

        if callback:
            qt_button.clicked.connect(callback)

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


def get_image_path(img_name):
    """Create an absolute image path to an image."""
    return os.path.join(
        rospkg.RosPack().get_path('march_rqt_input_device'),
        'resource',
        'img{0}'.format(img_name))


def check_string(text):
    """Split text into new lines on every third word.

    :type text: str
    :param text: The text to split
    :return New string which contains newlines
    """
    words = enumerate(text.replace('_', ' ').split(' '))
    return reduce(lambda acc, (i, x): acc + '\n' + x if i % 3 == 0 else acc + ' ' + x, words, '')[1:]
