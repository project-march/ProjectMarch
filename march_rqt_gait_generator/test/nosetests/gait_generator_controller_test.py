#!/usr/bin/env python
import unittest

from mock import Mock
import rospkg
from urdf_parser_py import urdf


from march_rqt_gait_generator.gait_generator_controller import GaitGeneratorController
from march_rqt_gait_generator.model.modifiable_subgait import ModifiableSubgait

PKG = 'march_rqt_gait_generator'


class GaitGeneratorControllerTest(unittest.TestCase):
    def setUp(self):
        self.gait_generator_view = Mock()
        self.gait_generator_view.topic_name_line_edit.text = Mock(return_value='/march/my_fancy_topic')
        self.gait_generator_view.joint_widgets.__getitem__ = Mock()

        self.robot = urdf.Robot.from_xml_file(rospkg.RosPack().get_path('march_description') + '/urdf/march4.urdf')
        self.gait_generator_controller = GaitGeneratorController(self.gait_generator_view, self.robot)

        self.duration = self.gait_generator_controller.subgait.duration
        self.num_joints = len(self.gait_generator_controller.subgait.joints)

        self.gait_name = 'walk'
        self.subgait_name = 'left_swing'
        self.version = 'MV_walk_leftswing_v2'
        self.resources_folder = rospkg.RosPack().get_path('march_rqt_gait_generator') + '/test/resources'
        self.subgait_path = '{rsc}/{gait}/{subgait}/{version}.subgait'.format(rsc=self.resources_folder,
                                                                              gait=self.gait_name,
                                                                              subgait=self.subgait_name,
                                                                              version=self.version)

    def test_init_load_gui_call(self):
        self.gait_generator_view.load_gait_into_ui.assert_called_once_with(self.gait_generator_controller.subgait)

    def test_init_empty_subgait_length(self):
        self.assertEqual(len(self.gait_generator_controller.subgait.joints), self.num_joints)

    # connect_buttons tests
    def test_connect_import_gait_button_call(self):
        self.gait_generator_view.import_gait_button.clicked.\
            connect.assert_called_once_with(self.gait_generator_controller.import_gait)

    def test_connect_export_gait_button_call(self):
        self.gait_generator_view.export_gait_button.clicked. \
            connect.assert_called_once_with(self.gait_generator_controller.export_gait)

    def test_connect_gait_type_combo_box_call(self):
        self.gait_generator_view.gait_type_combo_box.currentTextChanged.connect.assert_called_once()

    def test_connect_gait_name_line_edit_call(self):
        self.gait_generator_view.gait_name_line_edit.textChanged.connect.assert_called_once()

    def test_connect_version_name_line_edit_call(self):
        self.gait_generator_view.version_name_line_edit.textChanged.connect.assert_called_once()

    def test_connect_subgait_name_line_edit_call(self):
        self.gait_generator_view.subgait_name_line_edit.textChanged.connect.assert_called_once()

    def test_connect_duration_spin_box_call(self):
        self.gait_generator_view.duration_spin_box.valueChanged. \
            connect.assert_called_once_with(self.gait_generator_controller.update_gait_duration)

    def test_connect_time_slider_call(self):
        self.gait_generator_view.time_slider.valueChanged.connect.assert_called_once()

    # connect_plot tests
    def test_connect_plot_sig_plot_changed_call(self):
        self.assertEqual(self.gait_generator_view.joint_widgets[''].Plot.getItem(0, 0).plot_item.sigPlotChanged.
                         connect.call_count, self.num_joints)

    def test_connect_plot_add_setpoint_call(self):
        self.assertEqual(self.gait_generator_view.joint_widgets[''].Plot.getItem(0, 0).add_setpoint.
                         connect.call_count, self.num_joints)

    def test_connect_plot_remove_setpoint_call(self):
        self.assertEqual(self.gait_generator_view.joint_widgets[''].Plot.getItem(0, 0).add_setpoint.
                         connect.call_count, self.num_joints)

    def test_connect_plot_table_changed_call(self):
        self.assertEqual(self.gait_generator_view.joint_widgets[''].Table.itemChanged.connect.call_count,
                         self.num_joints)

    # set_playback_speed test
    def test_set_playback_speed(self):
        self.gait_generator_controller.set_playback_speed(50)
        self.assertEqual(self.gait_generator_controller.playback_speed, 50)

    # set_playback_speed test
    def test_set_current_time(self):
        self.gait_generator_controller.set_current_time(50)
        self.assertEqual(self.gait_generator_controller.current_time, 50)

    # start_time_slider_thread tests
    def test_start_time_slider_while_it_is_running(self):
        self.gait_generator_controller.time_slider_thread = 1
        self.gait_generator_controller.start_time_slider_thread()
        self.gait_generator_view.create_time_slider_thread.assert_not_called()

    def test_start_time_slider_thread(self):
        self.gait_generator_controller.start_time_slider_thread()
        self.gait_generator_view.create_time_slider_thread.assert_called_once()
        self.gait_generator_view.create_time_slider_thread().update_signal.connect.assert_called_once()
        self.gait_generator_view.create_time_slider_thread().start.assert_called_once()

    # stop_time_slider_thread tests
    def test_stop_time_slider_thread_not_running(self):
        self.gait_generator_controller.stop_time_slider_thread()
        self.gait_generator_view.create_time_slider_thread().stop.assert_not_called()

    def test_stop_time_slider_thread(self):
        self.gait_generator_controller.start_time_slider_thread()
        self.gait_generator_controller.stop_time_slider_thread()
        self.gait_generator_view.create_time_slider_thread().stop.assert_called_once()

    # update_gait_duration tests
    def test_update_gait_duration_no_rescale_one_setpoint_before_duration(self):
        self.gait_generator_view.scale_setpoints_check_box.isChecked = Mock(return_value=False)
        self.gait_generator_controller.update_gait_duration(self.duration / 2.0)

        self.gait_generator_view.message.assert_called_once_with(title='Could not update gait duration',
                                                                 msg='Not all joints have multiple setpoints before '
                                                                     'duration ' + str(self.duration / 2.0))
        self.gait_generator_view.duration_spin_box.setValue.assert_called_once_with(self.duration)
        self.assertEqual(self.gait_generator_controller.subgait.duration, self.duration)

    def test_update_gait_duration_no_rescale_dont_discard_setpoints(self):
        self.gait_generator_view.scale_setpoints_check_box.isChecked = Mock(return_value=False)
        self.gait_generator_view.yes_no_question = Mock(return_value=False)
        for joint in self.gait_generator_controller.subgait.joints:
            joint.add_interpolated_setpoint(self.duration / 2.0)
        self.gait_generator_controller.update_gait_duration(self.duration / 2.0)

        self.gait_generator_view.duration_spin_box.setValue.assert_called_once_with(self.duration)
        self.assertEqual(self.gait_generator_controller.subgait.duration, self.duration)

    def test_update_gait_duration_no_rescale_do_discard_setpoints(self):
        self.gait_generator_view.scale_setpoints_check_box.isChecked = Mock(return_value=False)
        self.gait_generator_view.yes_no_question = Mock(return_value=True)
        for joint in self.gait_generator_controller.subgait.joints:
            joint.add_interpolated_setpoint(self.duration / 2.0)
        self.gait_generator_controller.update_gait_duration(self.duration / 2.0)

        self.assertEqual(self.gait_generator_controller.subgait.duration, self.duration / 2.0)

    def test_update_gait_duration_no_rescale_longer_gait(self):
        self.gait_generator_view.scale_setpoints_check_box.isChecked = Mock(return_value=False)
        self.gait_generator_controller.update_gait_duration(self.duration + 1)
        self.assertEqual(self.gait_generator_controller.subgait.duration, self.duration + 1)

    def test_update_gait_duration_rescale_shorter_gait(self):
        self.gait_generator_view.scale_setpoints_check_box.isChecked = Mock(return_value=True)
        self.gait_generator_controller.update_gait_duration(self.duration / 2.0)
        self.assertEqual(self.gait_generator_controller.subgait.duration, self.duration / 2.0)

    def test_update_gait_duration_rescale_longer_gait(self):
        self.gait_generator_view.scale_setpoints_check_box.isChecked = Mock(return_value=True)
        self.gait_generator_controller.update_gait_duration(self.duration * 2.0)
        self.assertEqual(self.gait_generator_controller.subgait.duration, self.duration * 2.0)

    # import_gait tests
    def test_import_gait_wrong_gait_file(self):
        wrong_gait_name = 'this_gait_does_not_exist'
        wrong_subgait_path = '{rsc}/{gait}/{subgait}/{version}.subgait'.format(rsc=self.resources_folder,
                                                                               gait=wrong_gait_name,
                                                                               subgait=self.subgait_name,
                                                                               version=self.version)
        self.gait_generator_view.open_file_dialogue = Mock(return_value=(wrong_subgait_path, None))

        subgait = self.gait_generator_controller.subgait
        self.gait_generator_controller.import_gait()
        self.assertEqual(self.gait_generator_controller.subgait, subgait)

    def test_import_gait(self):
        self.gait_generator_view.open_file_dialogue = Mock(return_value=(self.subgait_path, None))

        self.gait_generator_controller.import_gait()
        self.assertEqual(self.gait_generator_controller.subgait.subgait_name, 'left_swing')

    def test_import_gait_load_gait_into_ui(self):
        self.gait_generator_view.open_file_dialogue = Mock(return_value=(self.subgait_path, None))

        self.gait_generator_controller.import_gait()
        self.gait_generator_view.load_gait_into_ui.assert_called_with(self.gait_generator_controller.subgait)

    def test_import_gait_change_gait_directory(self):
        self.gait_generator_view.open_file_dialogue = Mock(return_value=(self.subgait_path, None))

        self.gait_generator_controller.import_gait()
        self.assertEqual(self.gait_generator_controller.gait_directory, self.resources_folder)

    # export_to_file tests
    def test_export_to_file_new(self):
        import os

        self.gait_generator_view.open_file_dialogue = Mock(return_value=(self.subgait_path, None))
        self.gait_generator_controller.import_gait()

        new_subgait_name = 'shiny_new_subgait'
        self.gait_generator_controller.subgait.subgait_name = new_subgait_name
        new_subgait_path = '{rsc}/{gait}/{subgait}/{version}.subgait'.format(rsc=self.resources_folder,
                                                                             gait=self.gait_name,
                                                                             subgait=new_subgait_name,
                                                                             version=self.version)
        self.gait_generator_controller.export_to_file(self.gait_generator_controller.subgait, self.resources_folder)

        self.gait_generator_controller.subgait = ModifiableSubgait.empty_subgait(self, self.robot)
        self.gait_generator_view.open_file_dialogue = Mock(return_value=(new_subgait_path, None))
        self.gait_generator_controller.import_gait()
        self.assertEqual(self.gait_generator_controller.subgait.subgait_name, new_subgait_name,
                         msg='Name of subgait does not get set when importing the just exported subgait file.')
        os.remove(new_subgait_path)
        os.rmdir('{rsc}/{gait}/{subgait}'.format(rsc=self.resources_folder, gait=self.gait_name,
                                                 subgait=new_subgait_name))

    def test_export_to_file_existing_answer_yes(self):
        self.gait_generator_view.yes_no_question = Mock(return_value=True)
        self.gait_generator_view.open_file_dialogue = Mock(return_value=(self.subgait_path, None))
        self.gait_generator_controller.import_gait()
        self.gait_generator_controller.export_to_file(self.gait_generator_controller.subgait, self.resources_folder)

        self.gait_generator_view.notify.assert_called_once_with('Gait Saved', self.subgait_path)

    def test_export_to_file_existing_answer_no(self):
        self.gait_generator_view.yes_no_question = Mock(return_value=False)
        self.gait_generator_view.open_file_dialogue = Mock(return_value=(self.subgait_path, None))
        self.gait_generator_controller.import_gait()
        self.gait_generator_controller.export_to_file(self.gait_generator_controller.subgait, self.resources_folder)

        self.gait_generator_view.notify.assert_not_called()

    # export_gait tests
    def test_export_gait_no_mirror(self):
        self.gait_generator_view.mirror_check_box.isChecked = Mock(return_value=False)

        self.gait_generator_view.yes_no_question = Mock(return_value=True)
        self.gait_generator_view.open_file_dialogue = Mock(return_value=(self.subgait_path, None))
        self.gait_generator_controller.import_gait()
        self.gait_generator_controller.export_gait()

        self.gait_generator_view.notify.assert_called_once_with('Gait Saved', self.subgait_path)

    def test_export_gait_mirror(self):
        self.gait_generator_view.mirror_check_box.isChecked = Mock(return_value=True)
        self.gait_generator_view.mirror_key1_line_edit.text = Mock(return_value='left')
        self.gait_generator_view.mirror_key2_line_edit.text = Mock(return_value='right')

        self.gait_generator_view.yes_no_question = Mock(return_value=True)
        self.gait_generator_view.open_file_dialogue = Mock(return_value=(self.subgait_path, None))
        self.gait_generator_controller.import_gait()
        self.gait_generator_controller.export_gait()

        self.assertEqual(self.gait_generator_view.notify.call_count, 2)

    def test_export_gait_mirror_not_allowed(self):
        self.gait_generator_view.mirror_check_box.isChecked = Mock(return_value=True)
        self.gait_generator_view.mirror_key1_line_edit.text = Mock(return_value=None)
        self.gait_generator_view.mirror_key2_line_edit.text = Mock(return_value=None)

        self.gait_generator_view.yes_no_question = Mock(return_value=True)
        self.gait_generator_view.open_file_dialogue = Mock(return_value=(self.subgait_path, None))
        self.gait_generator_controller.import_gait()
        self.gait_generator_controller.export_gait()

        self.gait_generator_view.notify.assert_called_once_with('Could not mirror gait',
                                                                'Check the logs for more information.')

    # change_gait_directory tests
    def test_change_gait_directory(self):
        self.gait_generator_view.open_directory_dialogue = Mock(return_value='some/test/directory')
        self.gait_generator_controller.change_gait_directory()
        self.assertEqual(self.gait_generator_controller.gait_directory, 'some/test/directory')

    def test_change_gait_directory_canceled(self):
        self.gait_generator_controller.gait_directory = 'some/test/directory'
        self.gait_generator_view.open_directory_dialogue = Mock(return_value='')
        self.gait_generator_controller.change_gait_directory()
        self.assertIsNone(self.gait_generator_controller.gait_directory)

    # get_gait_directory tests
    def test_get_gait_directory(self):
        self.gait_generator_controller.gait_directory = 'some/test/directory'
        self.assertEqual(self.gait_generator_controller.get_gait_directory(), 'some/test/directory')

    def test_get_gait_directory_none(self):
        self.gait_generator_controller.get_gait_directory()
        self.gait_generator_view.open_directory_dialogue.assert_called_once()

    # invert_gait tests
    def test_invert_gait(self):
        self.gait_generator_controller.invert_gait()
        self.assertEqual(self.gait_generator_view.update_joint_widget.call_count, self.num_joints)

    # save_changed_joints tests
    def test_save_changed_joints(self):
        joint = self.gait_generator_controller.subgait.joints[0]
        self.gait_generator_controller.save_changed_joints([joint])
        self.assertEqual(self.gait_generator_controller.joint_changed_history.pop()[0], joint)

    def test_save_changed_joints_reset_redo(self):
        joint = self.gait_generator_controller.subgait.joints[0]
        self.gait_generator_controller.joint_changed_redo_list.append([joint])
        self.gait_generator_controller.save_changed_joints([joint])
        self.assertEqual(len(self.gait_generator_controller.joint_changed_redo_list), 0)

    # undo tests
    def test_undo_no_history(self):
        self.gait_generator_controller.undo()
        self.gait_generator_view.publish_preview.assert_not_called()

    def test_undo_change(self):
        joint = self.gait_generator_controller.subgait.joints[0]
        joint.add_interpolated_setpoint(self.duration / 2.0)
        self.gait_generator_controller.joint_changed_history.append([joint])
        self.gait_generator_controller.undo()
        self.assertEqual(len(joint.setpoints), 2)

    def test_undo_publish_preview(self):
        joint = self.gait_generator_controller.subgait.joints[0]
        joint.add_interpolated_setpoint(self.duration / 2.0)
        self.gait_generator_controller.joint_changed_history.append([joint])
        self.gait_generator_controller.undo()
        self.gait_generator_view.publish_preview.assert_called_once()

    # redo tests
    def test_redo_no_history(self):
        self.gait_generator_controller.redo()
        self.gait_generator_view.publish_preview.assert_not_called()

    def test_redo(self):
        joint = self.gait_generator_controller.subgait.joints[0]
        joint.add_interpolated_setpoint(self.duration / 2.0)
        self.gait_generator_controller.joint_changed_history.append([joint])
        self.gait_generator_controller.undo()
        self.gait_generator_controller.redo()
        self.assertEqual(len(joint.setpoints), 3)

    def test_redo_publish_preview(self):
        joint = self.gait_generator_controller.subgait.joints[0]
        joint.add_interpolated_setpoint(self.duration / 2.0)
        self.gait_generator_controller.joint_changed_history.append([joint])
        self.gait_generator_controller.undo()
        self.gait_generator_controller.redo()
        self.assertEqual(self.gait_generator_view.publish_preview.call_count, 2)
