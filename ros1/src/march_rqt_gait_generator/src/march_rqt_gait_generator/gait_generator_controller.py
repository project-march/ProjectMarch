import os

from numpy_ringbuffer import RingBuffer
import rospkg
import rospy

from march_shared_classes.gait.subgait import Subgait

from .model.modifiable_setpoint import ModifiableSetpoint
from .model.modifiable_subgait import ModifiableSubgait
from .side_subgait_controller import SideSubgaitController


class GaitGeneratorController(object):

    def __init__(self, view, robot):
        self.view = view

        # Default values
        self.gait_directory = None

        self.playback_speed = 100
        self.time_slider_thread = None
        self.current_time = 0
        self.robot = robot

        empty_subgait_file = os.path.join(rospkg.RosPack().get_path('march_rqt_gait_generator'),
                                          'resource/empty_gait/empty_subgait/empty_subgait.subgait')
        self.subgait = ModifiableSubgait.from_file(robot, empty_subgait_file, self)
        self.settings_changed_history = RingBuffer(capacity=100, dtype=list)
        self.settings_changed_redo_list = RingBuffer(capacity=100, dtype=list)

        standing = Subgait.from_file(robot, empty_subgait_file)
        previous_subgait_controller = SideSubgaitController(view=self.view.side_subgait_view['previous'],
                                                            default=standing)
        next_subgait_controller = SideSubgaitController(view=self.view.side_subgait_view['next'], default=standing)
        self.side_subgait_controller = {'previous': previous_subgait_controller, 'next': next_subgait_controller}

        self.connect_buttons()
        self.view.load_gait_into_ui(self.subgait)
        for joint in self.subgait.joints:
            self.connect_plot(joint)

    # Called by __init__
    def connect_buttons(self):
        self.view.change_gait_directory_button.clicked.connect(self.change_gait_directory)
        self.view.import_gait_button.clicked.connect(self.import_gait)
        self.view.export_gait_button.clicked.connect(self.export_gait)

        self.view.side_subgait_view['previous'].import_button.clicked.connect(
            lambda: self.import_side_subgait('previous'))
        self.view.side_subgait_view['previous'].default_checkbox.stateChanged.connect(
            lambda value: self.toggle_side_subgait_checkbox(value, 'previous', 'standing'))
        self.view.side_subgait_view['previous'].lock_checkbox.stateChanged.connect(
            lambda value: self.toggle_side_subgait_checkbox(value, 'previous', 'lock'))

        self.view.side_subgait_view['next'].import_button.clicked.connect(
            lambda: self.import_side_subgait('next'))
        self.view.side_subgait_view['next'].default_checkbox.stateChanged.connect(
            lambda value: self.toggle_side_subgait_checkbox(value, 'next', 'standing'))
        self.view.side_subgait_view['next'].lock_checkbox.stateChanged.connect(
            lambda value: self.toggle_side_subgait_checkbox(value, 'next', 'lock'))

        self.view.start_button.clicked.connect(self.start_time_slider_thread)
        self.view.stop_button.clicked.connect(self.stop_time_slider_thread)
        self.view.invert_button.clicked.connect(self.invert_gait)
        self.view.undo_button.clicked.connect(self.undo)
        self.view.redo_button.clicked.connect(self.redo)
        self.view.ctrl_z.activated.connect(self.undo)
        self.view.ctrl_shift_z.activated.connect(self.redo)

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
                joint.set_setpoints(joint_plot.to_setpoints()),
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
                joint.save_setpoints(),
                self.save_changed_settings({'joints': [joint]}),
                joint.set_setpoints(joint_widget.Table.controller.to_setpoints()),
                self.view.update_joint_widget(joint, update_table=False),
                self.view.publish_preview(self.subgait, self.current_time),
            ])

    # Functions below are connected to buttons, text boxes, joint graphs etc.
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
        self.time_slider_thread = self.view.create_time_slider_thread(current, playback_speed, max_time)
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
                self.view.set_duration_spinbox(self.subgait.duration)
                return
            discard_setpoints = self.view.yes_no_question(title='Gait duration lower than highest time setpoint',
                                                          msg='Do you want to discard any setpoints higher than the '
                                                          'given duration?')
            if not discard_setpoints:
                self.view.set_duration_spinbox(self.subgait.duration)
                return

        for joint in self.subgait.joints:
            joint.save_setpoints(single_joint_change=False)
        self.save_changed_settings({'joints': self.subgait.joints})

        self.subgait.scale_timestamps_subgait(duration, rescale_setpoints)
        self.view.time_slider.setRange(0, 100 * self.subgait.duration)

        was_playing = self.time_slider_thread is not None
        self.stop_time_slider_thread()

        self.view.update_joint_widgets(self.subgait.joints)

        if was_playing:
            self.start_time_slider_thread()

    def import_gait(self):
        file_name, f = self.view.open_file_dialogue()

        if file_name != '':
            gait = ModifiableSubgait.from_file(self.robot, file_name, self)
        else:
            gait = None

        if gait is None:
            rospy.logwarn('Could not load gait %s', file_name)
            return
        if gait.gait_type is None or gait.gait_type == '':
            gait.gait_type = 'walk_like'
        self.subgait = gait

        was_playing = self.time_slider_thread is not None
        self.stop_time_slider_thread()

        self.view.load_gait_into_ui(self.subgait)
        for joint in self.subgait.joints:
            self.connect_plot(joint)
        self.current_time = 0

        if was_playing:
            self.start_time_slider_thread()

        self.gait_directory = '/'.join(file_name.split('/')[:-3])
        rospy.loginfo('Setting gait directory to %s', str(self.gait_directory))
        self.view.change_gait_directory_button.setText(self.gait_directory)

        # Clear undo and redo history
        self.settings_changed_history = RingBuffer(capacity=100, dtype=list)
        self.settings_changed_redo_list = RingBuffer(capacity=100, dtype=list)

    def import_side_subgait(self, side='previous'):
        file_name, f = self.view.open_file_dialogue()

        if file_name != '':
            subgait = ModifiableSubgait.from_file(self.robot, file_name, self)
        else:
            subgait = None

        if subgait is None:
            rospy.logwarn('Could not load gait %s', file_name)
            return

        if side == 'previous':
            self.previous_subgait = subgait
        elif side == 'next':
            self.next_subgait = subgait

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
            output_file.write(subgait.to_yaml())

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
        for side, controller in self.side_subgait_controller.items():
            controller.lock_checked = False
            self.handle_sidepoint_lock(side)

        for joint in self.subgait.joints:
            joint.invert()
            self.view.update_joint_widget(joint)
        self.save_changed_settings({'joints': self.subgait.joints,
                                    'side_subgaits': self.side_subgait_controller.values()})
        self.view.publish_preview(self.subgait, self.current_time)

    def undo(self):
        if not self.settings_changed_history:
            return

        changed_dict = self.settings_changed_history.pop()

        if 'joints' in changed_dict:
            joints = changed_dict['joints']
            for joint in joints:
                joint.undo()
            self.subgait.scale_timestamps_subgait(joints[0].setpoints[-1].time, rescale=False)
            self.view.set_duration_spinbox(self.subgait.duration)

        if 'side_subgaits' in changed_dict:
            side_subgait_controllers = changed_dict['side_subgaits']
            for controller in side_subgait_controllers:
                controller.undo()

        self.view.update_joint_widgets(self.subgait.joints)
        self.view.publish_preview(self.subgait, self.current_time)
        self.settings_changed_redo_list.append(changed_dict)

    def redo(self):
        if not self.settings_changed_redo_list:
            return

        changed_dict = self.settings_changed_redo_list.pop()

        if 'joints' in changed_dict:
            joints = changed_dict['joints']
            for joint in joints:
                joint.redo()
            self.subgait.scale_timestamps_subgait(joints[0].setpoints[-1].time, rescale=False)
            self.view.set_duration_spinbox(self.subgait.duration)

        if 'side_subgaits' in changed_dict:
            side_subgait_controllers = changed_dict['side_subgaits']
            for controller in side_subgait_controllers:
                controller.redo()

        self.view.update_joint_widgets(self.subgait.joints)
        self.view.publish_preview(self.subgait, self.current_time)
        self.settings_changed_history.append(changed_dict)

    # Needed for undo and redo.
    def save_changed_settings(self, settings):
        self.settings_changed_history.append(settings)
        self.settings_changed_redo_list = RingBuffer(capacity=100, dtype=list)

    # Functions related to previous/next subgait
    def toggle_side_subgait_checkbox(self, value, side, box_type):
        self.save_changed_settings({'joints': self.subgait.joints,
                                    'side_subgaits': [self.side_subgait_controller[side]]})
        for joint in self.subgait.joints:
            joint.save_setpoints(single_joint_change=False)
        if box_type == 'lock':
            self.side_subgait_controller[side].lock_checked = value
        elif box_type == 'standing':
            self.side_subgait_controller[side].default_checked = value
        self.handle_sidepoint_lock(side)

    def handle_sidepoint_lock(self, side):
        if self.side_subgait_controller[side].lock_checked:
            if self.side_subgait_controller[side].subgait is not None:
                for joint in self.subgait.joints:
                    side_subgait_joint = self.side_subgait_controller[side].subgait.get_joint(joint.name)
                    if side == 'previous':
                        joint.start_point = side_subgait_joint.setpoints[-1]
                    elif side == 'next':
                        joint.end_point = side_subgait_joint.setpoints[0]
            else:
                for joint in self.subgait.joints:
                    if side == 'previous':
                        joint.start_point = joint.setpoints[0]
                    elif side == 'next':
                        joint.end_point = joint.setpoints[-1]
        else:
            for joint in self.subgait.joints:
                if side == 'previous':
                    joint.start_point = None
                elif side == 'next':
                    joint.end_point = None

        self.view.update_joint_widgets(self.subgait.joints)
        self.view.publish_preview(self.subgait, self.current_time)

    @property
    def previous_subgait(self):
        return self.side_subgait_controller['previous'].subgait

    @previous_subgait.setter
    def previous_subgait(self, new_subgait):
        self.save_changed_settings({'joints': self.subgait.joints,
                                    'side_subgaits': [self.side_subgait_controller['previous']]})
        for joint in self.subgait.joints:
            joint.save_setpoints(single_joint_change=False)
        self.side_subgait_controller['previous'].subgait = new_subgait
        self.handle_sidepoint_lock('previous')

    @property
    def next_subgait(self):
        return self.side_subgait_controller['next'].subgait

    @next_subgait.setter
    def next_subgait(self, new_subgait):
        self.save_changed_settings({'joints': self.subgait.joints,
                                    'side_subgaits': [self.side_subgait_controller['next']]})
        for joint in self.subgait.joints:
            joint.save_setpoints(single_joint_change=False)
        self.side_subgait_controller['next'].subgait = new_subgait
        self.handle_sidepoint_lock('next')
