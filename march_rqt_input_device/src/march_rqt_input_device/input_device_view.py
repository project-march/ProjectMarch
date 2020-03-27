import os


from python_qt_binding import loadUi
from python_qt_binding.QtCore import QSize
from python_qt_binding.QtWidgets import QGridLayout
from python_qt_binding.QtWidgets import QPushButton
from python_qt_binding.QtWidgets import QWidget
from qt_gui.plugin import Plugin
import rospkg

from march_rqt_input_device.input_device_controller import InputDeviceController


class InputDevicePlugin(Plugin):
    def __init__(self, context):
        super(InputDevicePlugin, self).__init__(context)
        self.controller = InputDeviceController()

        # region initialising the widget
        self.setObjectName('InputDevicePlugin')

        # Create QWidget
        self._widget = QWidget()

        # Get path to UI file which should be in the 'resource' folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('march_rqt_input_device'), 'resource', 'input_device.ui')

        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)

        # Give QObjects reasonable names
        self._widget.setObjectName('Input Device')

        # Show _widget.windowTitle on left-top of each plugin (when it's set in _widget). (useful for multiple windows)
        if context.serial_number() > 1:
            self._widget.setWindowTitle('{0} ({1})'.format(self._widget.windowTitle(), context.serial_number()))

        # Add widget to the user interface
        context.add_widget(self._widget)

        # Create buttons here
        rocker_switch_increment = \
            self.create_button('rocker_switch_up', image_path='/rocker_switch_up.png',
                               callback=lambda: self.controller.publish_increment_step_size())

        rocker_switch_decrement = \
            self.create_button('rocker_switch_down', image_path='/rocker_switch_down.png',
                               callback=lambda: self.controller.publish_decrement_step_size())

        home_sit = \
            self.create_button('home_sit', image_path='/home_sit.png',
                               callback=lambda: self.controller.publish_gait('home_sit'))

        home_stand = \
            self.create_button('home_stand', image_path='/home_stand.png',
                               callback=lambda: self.controller.publish_gait('home_stand'))

        gait_sit = \
            self.create_button('gait_sit', image_path='/gait_sit.png',
                               callback=lambda: self.controller.publish_gait('gait_sit'))

        gait_walk = \
            self.create_button('gait_walk', image_path='/gait_walk.png',
                               callback=lambda: self.controller.publish_gait('gait_walk'))

        gait_walk_small = \
            self.create_button('gait_walk_small', image_path='/gait_walk_small.png',
                               callback=lambda: self.controller.publish_gait('gait_walk_small'))

        gait_walk_large = \
            self.create_button('gait_walk_large', image_path='/gait_walk_large.png',
                               callback=lambda: self.controller.publish_gait('gait_walk_large'))

        gait_single_step_small = \
            self.create_button('gait_single_step_small', image_path='/gait_single_step_small.png',
                               callback=lambda: self.controller.publish_gait('gait_single_step_small'))

        gait_single_step_normal = \
            self.create_button('gait_single_step_normal', image_path='/gait_single_step_medium.png',
                               callback=lambda: self.controller.publish_gait('gait_single_step_normal'))

        gait_side_step_left = \
            self.create_button('gait_side_step_left', image_path='/gait_side_step_left.png',
                               callback=lambda: self.controller.publish_gait('gait_side_step_left'))

        gait_side_step_right = \
            self.create_button('gait_side_step_right', image_path='/gait_side_step_right.png',
                               callback=lambda: self.controller.publish_gait('gait_side_step_right'))

        gait_side_step_left_small = \
            self.create_button('gait_side_step_left_small',
                               callback=lambda: self.controller.publish_gait('gait_side_step_left_small'))

        gait_side_step_right_small = \
            self.create_button('gait_side_step_right_small',
                               callback=lambda: self.controller.publish_gait('gait_side_step_right_small'))

        gait_stand = \
            self.create_button('gait_stand', image_path='/gait_stand.png',
                               callback=lambda: self.controller.publish_gait('gait_stand'))

        gait_sofa_stand = \
            self.create_button('gait_sofa_stand', image_path='/gait_sofa_stand_up.png',
                               callback=lambda: self.controller.publish_gait('gait_sofa_stand'))

        gait_sofa_sit = \
            self.create_button('gait_sofa_sit', image_path='/gait_sofa_sit.png',
                               callback=lambda: self.controller.publish_gait('gait_sofa_sit'))

        gait_stairs_up = \
            self.create_button('gait_stairs_up', image_path='/gait_stairs_up.png',
                               callback=lambda: self.controller.publish_gait('gait_stairs_up'))

        gait_stairs_down = \
            self.create_button('gait_stairs_down', image_path='/gait_stairs_down.png',
                               callback=lambda: self.controller.publish_gait('gait_stairs_down'))

        gait_stairs_up_single_step = \
            self.create_button('gait_stairs_up_single_step', image_path='/gait_stairs_up_single_step.png',
                               callback=lambda: self.controller.publish_gait('gait_stairs_up_single_step'))

        gait_stairs_down_single_step = \
            self.create_button('gait_stairs_down_single_step', image_path='/gait_stairs_down_single_step.png',
                               callback=lambda: self.controller.publish_gait('gait_stairs_down_single_step'))

        gait_rough_terrain_high_step = \
            self.create_button('gait_rough_terrain_high_step', image_path='/gait_rough_terrain_high_step.png',
                               callback=lambda: self.controller.publish_gait('gait_rough_terrain_high_step'))

        gait_rough_terrain_middle_steps = \
            self.create_button('gait_rough_terrain_middle_steps',
                               callback=lambda: self.controller.publish_gait('gait_rough_terrain_middle_steps'))

        gait_rough_terrain_first_middle_step = \
            self.create_button('gait_rough_terrain_first_middle_step',
                               callback=lambda: self.controller.publish_gait('gait_rough_terrain_first_middle_step'))

        gait_rough_terrain_second_middle_step = \
            self.create_button('gait_rough_terrain_second_middle_step',
                               callback=lambda: self.controller.publish_gait('gait_rough_terrain_second_middle_step'))

        gait_rough_terrain_third_middle_step = \
            self.create_button('gait_rough_terrain_third_middle_step',
                               callback=lambda: self.controller.publish_gait('gait_rough_terrain_third_middle_step'))

        gait_ramp_door_slope_up = \
            self.create_button('gait_ramp_door_slope_up',
                               callback=lambda: self.controller.publish_gait('gait_ramp_door_slope_up'))

        gait_ramp_door_slope_down = \
            self.create_button('gait_ramp_door_slope_down',
                               callback=lambda: self.controller.publish_gait('gait_ramp_door_slope_down'))

        gait_ramp_door_last_step = \
            self.create_button('gait_ramp_door_last_step',
                               callback=lambda: self.controller.publish_gait('gait_ramp_door_last_step'))

        gait_tilted_path_left_straight_start = \
            self.create_button('gait_tilted_path_left_straight_start',
                               callback=lambda: self.controller.publish_gait('gait_tilted_path_left_straight_start'))

        gait_tilted_path_left_single_step = \
            self.create_button('gait_tilted_path_left_single_step',
                               callback=lambda: self.controller.publish_gait('gait_tilted_path_left_single_step'))

        gait_tilted_path_left_straight_end = \
            self.create_button('gait_tilted_path_left_straight_end',
                               callback=lambda: self.controller.publish_gait('gait_tilted_path_left_straight_end'))

        gait_tilted_path_left_flexed_knee_step = \
            self.create_button('gait_tilted_path_left_flexed_knee_step',
                               callback=lambda: self.controller.publish_gait('gait_tilted_path_left_flexed_knee_step'))

        gait_tilted_path_right_straight_start = \
            self.create_button('gait_tilted_path_right_straight_start',
                               callback=lambda: self.controller.publish_gait('gait_tilted_path_right_straight_start'))

        gait_tilted_path_right_single_step = \
            self.create_button('gait_tilted_path_right_single_step',
                               callback=lambda: self.controller.publish_gait('gait_tilted_path_right_single_step'))

        gait_tilted_path_right_straight_end = \
            self.create_button('gait_tilted_path_right_straight_end',
                               callback=lambda: self.controller.publish_gait('gait_tilted_path_right_straight_end'))

        gait_tilted_path_right_flexed_knee_step = \
            self.create_button('gait_tilted_path_right_flexed_knee_step',
                               callback=lambda: self.controller.publish_gait('gait_tilted_path_right_flexed_knee_step'))

        gait_tilted_path_first_start = \
            self.create_button('gait_tilted_path_first_start',
                               callback=lambda: self.controller.publish_gait('gait_tilted_path_first_start'))

        gait_tilted_path_second_start = \
            self.create_button('gait_tilted_path_second_start',
                               callback=lambda: self.controller.publish_gait('gait_tilted_path_second_start'))

        gait_tilted_path_first_end = \
            self.create_button('gait_tilted_path_first_end',
                               callback=lambda: self.controller.publish_gait('gait_tilted_path_first_end'))

        gait_tilted_path_second_end = \
            self.create_button('gait_tilted_path_second_end',
                               callback=lambda: self.controller.publish_gait('gait_tilted_path_second_end'))

        stop_button = self.create_button('gait_stop', image_path='/stop.png',
                                         callback=lambda: self.controller.publish_stop())

        pause_button = self.create_button('gait_pause', image_path='/pause.png',
                                          callback=lambda: self.controller.publish_pause())

        continue_button = self.create_button('gait_continue', image_path='/continue.png',
                                             callback=lambda: self.controller.publish_continue())

        error_button = self.create_button('error', image_path='/error.png',
                                          callback=lambda: self.controller.publish_error())

        # The button layout.
        # Position in the array determines position on screen.
        march_button_layout = [

            [home_sit, home_stand, gait_walk, gait_walk_small, gait_walk_large],

            [gait_sit, gait_stand, rocker_switch_increment, rocker_switch_decrement, stop_button, error_button],

            [gait_sofa_sit, gait_sofa_stand, gait_single_step_normal, gait_single_step_small, continue_button, pause_button],

            [gait_stairs_up, gait_stairs_down, gait_stairs_up_single_step, gait_stairs_down_single_step],

            [gait_side_step_left, gait_side_step_right, gait_side_step_left_small, gait_side_step_right_small],

            [gait_rough_terrain_high_step, gait_rough_terrain_middle_steps, gait_rough_terrain_first_middle_step,
             gait_rough_terrain_second_middle_step, gait_rough_terrain_third_middle_step],

            [gait_ramp_door_slope_up, gait_ramp_door_slope_down, gait_ramp_door_last_step],

            [gait_tilted_path_left_straight_start, gait_tilted_path_left_single_step,
             gait_tilted_path_left_straight_end, gait_tilted_path_left_flexed_knee_step],

            [gait_tilted_path_right_straight_start, gait_tilted_path_right_single_step,
             gait_tilted_path_right_straight_end, gait_tilted_path_right_flexed_knee_step],

            [gait_tilted_path_first_start, gait_tilted_path_second_start, gait_tilted_path_first_end,
             gait_tilted_path_second_end],
        ]

        # Create the qt_layout from the button layout.
        qt_layout = self.create_layout(march_button_layout)

        # Apply the qt_layout to the top level widget.
        self._widget.frame.findChild(QWidget, 'content').setLayout(qt_layout)

        # Make the frame as tight as possible with spacing between the buttons.
        qt_layout.setSpacing(15)
        self._widget.frame.findChild(QWidget, 'content').adjustSize()

    @staticmethod
    def create_button(text, callback=None, image_path=None, size=(125, 150), visible=True, color_code='#1F1E24'):
        """Create a push button which the mock input device can register.

        :param text:
            Possible name of the button
        :param callback:
            The callback to attach to the button when pressed
        :param image_path:
            The name of the image file
        :param size:
            Default size of the button
        :param visible:
            Turn button invisible on the gui
        :param color_code:
            Possible background color of the button

        :return:
            A QPushButton object which contains the passed arguments
        """
        qt_button = QPushButton()

        if image_path:
            qt_button.setStyleSheet(create_image_button_css(get_image_path(image_path)))
        else:
            text = check_string(text)
            qt_button.setStyleSheet(create_color_button_css(color_code))
            qt_button.setText(text)

        qt_button.setMinimumSize(QSize(*size))

        if not visible:
            qt_button.setVisible(False)

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


def create_image_button_css(img_path):
    """CSS of a button with a background-image."""
    css_base = """
    background: url(<img_path>) no-repeat center center;
    background-color:#1F1E24;
    color: #000000;
    """
    return css_base.replace('<img_path>', img_path)


def create_color_button_css(color_code):
    """CSS of a button with a background-color."""
    css_base = """
    background-color: <color_code>;
    color: #FFFFFF;
    """
    return css_base.replace('<color_code>', color_code)


def get_image_path(img_name):
    """Create an absolute image path to an image."""
    return os.path.join(
        rospkg.RosPack().get_path('march_rqt_input_device'),
        'resource',
        'img{0}'.format(img_name))


def check_string(text):
    """Split text into two lines if sentence is to long.

    :param text:
        The text to split in half.

    :return:
        New string which contains of an enter in the middle.
    """
    text = text.replace('_', ' ')
    split_text = text.split(' ')
    if len(split_text) >= 3:

        new_string = ''
        middle = len(split_text) / 2

        new_string += ' '.join(split_text[:middle])
        new_string += '\n'
        new_string += ' '.join(split_text[middle:])
        return new_string

    else:
        return text
