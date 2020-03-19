import os

from numpy_ringbuffer import RingBuffer
import rospy
from trajectory_msgs.msg import JointTrajectory
from urdf_parser_py import urdf

from . import user_interface_controller
from .model.modifiable_setpoint import ModifiableSetpoint
from .model.modifiable_subgait import ModifiableSubgait
from .time_slider_thread import TimeSliderThread


class GaitGeneratorController(object):

    def __init__(self, view):
        self.view = view

        # Default values
        self.gait_publisher = None
        self.set_topic_name(self.view.topic_name_line_edit.text())
        self.gait_directory = None

        self.playback_speed = 100
        self.time_slider_thread = None
        self.current_time = 0

        self.robot = urdf.Robot.from_parameter_server()
        self.subgait = ModifiableSubgait.empty_subgait(self, self.robot)
        self.joint_changed_history = RingBuffer(capacity=100, dtype=list)
        self.joint_changed_redo_list = RingBuffer(capacity=100, dtype=list)

        self.connect_buttons()
        self.view.load_gait_into_ui(self.subgait)
        for joint in self.subgait.joints:
            self.connect_plot(joint)

    # Called by __init__
    def connect_buttons(self):
        self.view.change_gait_directory_button.clicked.connect(self.change_gait_directory)
        self.view.import_gait_button.clicked.connect(self.import_gait)
        self.view.export_gait_button.clicked.connect(self.export_gait)

        self.view.publish_gait_button.clicked.connect(self.publish_gait)

        self.view.start_button.clicked.connect(self.start_time_slider_thread)
        self.view.stop_button.clicked.connect(self.stop_time_slider_thread)
        self.view.invert_button.clicked.connect(self.invert_gait)
        self.view.undo_button.clicked.connect(self.undo)
        self.view.redo_button.clicked.connect(self.redo)

        # Line edits / combo boxes / spin boxes
        self.view.gait_type_combo_box.currentTextChanged.connect(
            lambda text: self.subgait.set_gait_type(text),
        )
        self.view.gait_name_line_edit.textChanged.connect(
            lambda text: self.subgait.set_gait_name(text),
        )
        self.view.version_name_line_edit.textChanged.connect(
            lambda text: self.subgait.set_version(text),
        )
        self.view.subgait_name_line_edit.textChanged.connect(
            lambda text: self.subgait.set_subgait_name(text),
        )
        self.view.description_line_edit.textChanged.connect(
            lambda text: self.subgait.set_description(text),
        )
        self.view.topic_name_line_edit.textChanged.connect(self.set_topic_name)
        self.view.playback_speed_spin_box.valueChanged.connect(self.set_playback_speed)
        self.view.duration_spin_box.setKeyboardTracking(False)
        self.view.duration_spin_box.valueChanged.connect(self.update_gait_duration)

        # Check boxes
        self.view.velocity_plot_check_box.stateChanged.connect(
            lambda: self.view.update_joint_widgets(self.subgait.joints),
        )
        self.view.effort_plot_check_box.stateChanged.connect(
            lambda: self.view.update_joint_widgets(self.subgait.joints),
        )
        # Disable key inputs when mirroring is off.
        self.view.mirror_check_box.stateChanged.connect(
            lambda state: [
                self.view.mirror_key1_line_edit.setEnabled(state),
                self.view.mirror_key2_line_edit.setEnabled(state),
            ])

        # Connect TimeSlider to the preview
        self.view.time_slider.valueChanged.connect(lambda time: [
            self.set_current_time(time / 100.0),
            self.view.publish_preview(self.subgait, self.current_time),
            self.view.update_time_sliders(self.current_time),
        ])

    # Called by __init__
    def connect_plot(self, joint):
        joint_widget = self.view.joint_widgets[joint.name]
        joint_plot = joint_widget.Plot.getItem(0, 0)

        def add_setpoint(joint, time, position, button):
            if button == self.view.control_button:
                joint.add_interpolated_setpoint(time)
            else:
                joint.add_setpoint(ModifiableSetpoint(time, position, 0))

        # Connect a function to update the model and to update the table.
        joint_plot.plot_item.sigPlotChanged.connect(
            lambda: [
                joint.set_setpoints(user_interface_controller.plot_to_setpoints(joint_plot)),
                self.view.update_joint_widget(joint),
                self.view.publish_preview(self.subgait, self.current_time),
            ])

        joint_plot.add_setpoint.connect(
            lambda time, position, button: [
                add_setpoint(joint, time, position, button),
                self.view.update_joint_widget(joint),
                self.view.publish_preview(self.subgait, self.current_time),
            ])

        joint_plot.remove_setpoint.connect(
            lambda index: [
                joint.remove_setpoint(index),
                self.view.update_joint_widget(joint),
                self.view.publish_preview(self.subgait, self.current_time),
            ])

        joint_widget.Table.itemChanged.connect(
            lambda: [
                joint.set_setpoints(user_interface_controller.table_to_setpoints(joint_widget.Table)),
                self.view.update_joint_widget(joint, update_table=False),
                self.view.publish_preview(self.subgait, self.current_time),
            ])

    # Functions below are connected to buttons, text boxes, joint graphs etc.
    def publish_gait(self):
        trajectory = self.subgait._to_joint_trajectory_msg()
        rospy.loginfo('Publishing trajectory to topic ' + self.topic_name)
        self.gait_publisher.publish(trajectory)

    def set_topic_name(self, topic_name):
        self.topic_name = topic_name
        self.gait_publisher = rospy.Publisher(topic_name,
                                              JointTrajectory, queue_size=10)

    def set_playback_speed(self, playback_speed):
        was_playing = self.time_slider_thread is not None
        self.stop_time_slider_thread()

        self.playback_speed = playback_speed
        rospy.loginfo('Changing playbackspeed to ' + str(self.playback_speed))

        if was_playing:
            self.start_time_slider_thread()

    def set_current_time(self, current_time):
        self.current_time = current_time

    def start_time_slider_thread(self):
        if self.time_slider_thread is not None:
            rospy.logdebug('Cannot start another time slider thread as one is already active')
            return

        current = self.view.time_slider.value()
        playback_speed = self.playback_speed
        max_time = self.view.time_slider.maximum()
        self.time_slider_thread = TimeSliderThread(current, playback_speed, max_time)
        self.time_slider_thread.update_signal.connect(self.view.update_main_time_slider)
        self.time_slider_thread.start()

    def stop_time_slider_thread(self):
        if self.time_slider_thread is not None:
            self.time_slider_thread.stop()
            self.time_slider_thread = None

    def update_gait_duration(self, duration):
        rescale_setpoints = self.view.scale_setpoints_check_box.isChecked()

        if self.subgait.has_setpoints_after_duration(duration) and not rescale_setpoints:
            if not self.subgait.has_multiple_setpoints_before_duration(duration):
                self.view.message(title='Could not update gait duration',
                                  msg='Not all joints have multiple setpoints before duration ' + str(duration))
                self.view.duration_spin_box.setValue(self.subgait.duration)
                return
            discard_setpoints = self.view.yes_no_question(title='Gait duration lower than highest time setpoint',
                                                          msg='Do you want to discard any setpoints higher than the '
                                                          'given duration?')
            if not discard_setpoints:
                self.view.duration_spin_box.setValue(self.subgait.duration)
                return
        self.subgait.set_duration(duration, rescale_setpoints)
        self.view.time_slider.setRange(0, 100 * self.subgait.duration)

        was_playing = self.time_slider_thread is not None
        self.stop_time_slider_thread()

        self.view.update_joint_widgets(self.subgait.joints)

        if was_playing:
            self.start_time_slider_thread()

    def import_gait(self):
        file_name, f = self.view.open_file_dialogue()

        gait = ModifiableSubgait.from_file(self.robot, file_name, self)
        if gait is None:
            rospy.logwarn('Could not load gait %s', file_name)
            return
        self.subgait = gait
        self.view.load_gait_into_ui(self.subgait)
        for joint in self.subgait.joints:
            self.connect_plot(joint)
        self.current_time = 0

        self.gait_directory = '/'.join(file_name.split('/')[:-3])
        rospy.loginfo('Setting gait directory to %s', str(self.gait_directory))
        self.view.change_gait_directory_button.setText(self.gait_directory)

        # Clear undo and redo history
        self.joint_changed_history = RingBuffer(capacity=100, dtype=list)
        self.joint_changed_redo_list = RingBuffer(capacity=100, dtype=list)

    def export_gait(self):
        if self.view.mirror_check_box.isChecked():
            key_1 = self.view.mirror_key1_line_edit.text()
            key_2 = self.view.mirror_key2_line_edit.text()
            mirror = self.subgait.get_mirror(key_1, key_2)

            if mirror:
                self.export_to_file(mirror, self.get_gait_directory())
            else:
                self.view.notify('Could not mirror gait', 'Check the logs for more information.')
                return

        self.export_to_file(self.subgait, self.get_gait_directory())

    def export_to_file(self, subgait, gait_directory):
        if gait_directory is None or gait_directory == '':
            return

        subgait_msg = subgait.to_subgait_msg()

        output_file_directory = os.path.join(gait_directory,
                                             subgait.gait_name.replace(' ', '_'),
                                             subgait.subgait_name.replace(' ', '_'))
        output_file_path = os.path.join(output_file_directory,
                                        subgait.version.replace(' ', '_') + '.subgait')

        file_exists = os.path.isfile(output_file_path)
        if file_exists:
            overwrite_file = self.view.yes_no_question(title='File already exists',
                                                       msg='Do you want to overwrite ' + str(output_file_path) + '?')
            if not overwrite_file:
                return

        rospy.loginfo('Writing gait to ' + output_file_path)

        if not os.path.isdir(output_file_directory):
            os.makedirs(output_file_directory)

        with open(output_file_path, 'w') as output_file:
            output_file.write(str(subgait_msg))

        self.view.notify('Gait Saved', output_file_path)

    # Called by export_gait
    def get_gait_directory(self):
        if self.gait_directory is None:
            self.change_gait_directory()
        rospy.loginfo('Selected output directory ' + str(self.gait_directory))
        return self.gait_directory

    def change_gait_directory(self):
        self.gait_directory = str(self.view.open_directory_dialogue())
        if self.gait_directory == '':   # If directory dialogue is canceled
            self.gait_directory = None
            self.view.change_gait_directory_button.setText('Select a gait directory...')
        else:
            self.view.change_gait_directory_button.setText(self.gait_directory)

    def invert_gait(self):
        for joint in self.subgait.joints:
            joint.invert()
            self.view.update_joint_widget(joint)
        self.save_changed_joints(self.subgait.joints)
        self.view.publish_preview(self.subgait, self.current_time)

    def undo(self):
        if not self.joint_changed_history:
            return

        joints = self.joint_changed_history.pop()
        for joint in joints:
            joint.undo()
            self.view.update_joint_widget(joint)
        self.joint_changed_redo_list.append(joints)
        self.view.publish_preview(self.subgait, self.current_time)

    def redo(self):
        if not self.joint_changed_redo_list:
            return

        joints = self.joint_changed_redo_list.pop()
        for joint in joints:
            joint.redo()
        self.joint_changed_history.append(joints)
        self.view.update_joint_widgets(joints)
        self.view.publish_preview(self.subgait, self.current_time)

    # Needed for undo and redo.
    def save_changed_joints(self, joints):
        self.joint_changed_history.append(joints)
        self.joint_changed_redo_list = RingBuffer(capacity=100, dtype=list)
