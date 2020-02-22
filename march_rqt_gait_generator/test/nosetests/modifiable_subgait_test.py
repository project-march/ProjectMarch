import copy
import unittest

from mock import Mock
import rospkg
from urdf_parser_py import urdf

from march_rqt_gait_generator.model.modifiable_setpoint import ModifiableSetpoint
from march_rqt_gait_generator.model.modifiable_subgait import ModifiableSubgait


class ModifiableSubgaitTest(unittest.TestCase):
    def setUp(self):
        self.gait_generator = Mock()
        self.gait_name = 'walk'
        self.subgait_name = 'left_swing'
        self.version = 'MV_walk_leftswing_v2'
        self.resources_folder = rospkg.RosPack().get_path('march_rqt_gait_generator') + '/test/resources'
        self.robot = urdf.Robot.from_xml_file(self.resources_folder + '/march4.urdf')
        self.subgait_path = '{rsc}/{gait}/{subgait}/{version}.subgait'.format(rsc=self.resources_folder,
                                                                              gait=self.gait_name,
                                                                              subgait=self.subgait_name,
                                                                              version=self.version)
        self.subgait = ModifiableSubgait.from_file(self.robot, self.subgait_path, self.gait_generator)
        self.subgait_msg = self.subgait.to_subgait_msg()

    # empty_subgait tests
    def test_empty_subgait_type(self):
        empty_subgait = ModifiableSubgait.empty_subgait(self.gait_generator, self.robot)
        self.assertIsInstance(empty_subgait, ModifiableSubgait)

    def test_empty_subgait_length(self):
        empty_subgait = ModifiableSubgait.empty_subgait(self.gait_generator, self.robot)
        number_of_joints = sum([1 for joint in self.robot.joints if joint.type == 'revolute'])
        self.assertEqual(len(empty_subgait.joints), number_of_joints)

    def test_empty_subgait_no_robot(self):
        empty_subgait = ModifiableSubgait.empty_subgait(self.gait_generator, None)
        self.assertIsNone(empty_subgait)

    # has_multiple_setpoints_before_duration tests
    def test_has_multiple_setpoints_before_duration_true(self):
        self.assertTrue(self.subgait.has_multiple_setpoints_before_duration(duration=self.subgait.duration + 1))

    def test_has_multiple_setpoints_before_duration_border(self):
        self.assertTrue(self.subgait.has_multiple_setpoints_before_duration(duration=self.subgait.duration))

    def test_has_multiple_setpoints_before_duration_false(self):
        self.assertFalse(self.subgait.has_multiple_setpoints_before_duration(duration=self.subgait.duration - 1))

    # has_setpoints_after_duration tests
    def test_has_setpoints_after_duration_true(self):
        self.assertTrue(self.subgait.has_setpoints_after_duration(duration=self.subgait.duration - 1))

    def test_has_setpoints_after_duration_border(self):
        self.assertFalse(self.subgait.has_setpoints_after_duration(duration=self.subgait.duration))

    def test_has_setpoints_after_duration_false(self):
        self.assertFalse(self.subgait.has_setpoints_after_duration(duration=self.subgait.duration + 1))

    # setters tests
    def test_set_gait_type(self):
        self.subgait.set_gait_type(u'banana type')
        self.assertEqual(self.subgait.gait_type, 'banana type')
        self.assertIsInstance(self.subgait.gait_type, str)

    def test_set_gait_name(self):
        self.subgait.set_gait_name(u'banana name')
        self.assertEqual(self.subgait.gait_name, 'banana name')
        self.assertIsInstance(self.subgait.gait_name, str)

    def test_set_description(self):
        self.subgait.set_description(u'banana description')
        self.assertEqual(self.subgait.description, 'banana description')
        self.assertIsInstance(self.subgait.description, str)

    def test_set_version(self):
        self.subgait.set_version(u'banana version')
        self.assertEqual(self.subgait.version, 'banana version')
        self.assertIsInstance(self.subgait.version, str)

    def test_set_subgait_name(self):
        self.subgait.set_subgait_name(u'banana name')
        self.assertEqual(self.subgait.subgait_name, 'banana name')
        self.assertIsInstance(self.subgait.subgait_name, str)

    # set_duration tests
    def test_set_duration_no_rescale_shorter(self):
        extra_setpoint = ModifiableSetpoint(0.4, 0, 0)
        self.subgait.get_joint('right_ankle').add_setpoint(extra_setpoint)
        self.subgait.get_joint('left_ankle').add_setpoint(extra_setpoint)
        self.subgait.get_joint('right_hip_fe').add_setpoint(extra_setpoint)
        self.subgait.set_duration(0.8, False)
        self.assertEqual(len(self.subgait.get_joint('left_knee').setpoints), 2)
        self.assertEqual(self.subgait.duration, 0.8)
        self.assertEqual([joint.duration for joint in self.subgait.joints], [0.8] * 8)

    def test_set_duration_no_rescale_longer(self):
        self.subgait.set_duration(1.8, False)
        self.assertEqual(self.subgait.duration, 1.8)
        self.assertEqual([joint.duration for joint in self.subgait.joints], [1.8] * 8)

    def test_set_duration_rescale_shorter(self):
        test_setpoint = copy.deepcopy(self.subgait.get_joint('left_knee').setpoints[2])
        test_setpoint.time = test_setpoint.time * 2
        new_duration = self.subgait.duration * 2
        self.subgait.set_duration(new_duration, True)
        self.assertEqual(self.subgait.get_joint('left_knee').setpoints[2], test_setpoint)
        self.assertEqual(self.subgait.duration, new_duration)
        self.assertEqual([joint.duration for joint in self.subgait.joints], [new_duration] * 8)

    def test_set_duration_rescale_longer(self):
        test_setpoint = copy.deepcopy(self.subgait.get_joint('left_knee').setpoints[2])
        test_setpoint.time = test_setpoint.time * 2
        new_duration = self.subgait.duration * 2
        self.subgait.set_duration(new_duration, True)
        self.assertEqual(self.subgait.get_joint('left_knee').setpoints[2], test_setpoint)
        self.assertEqual(self.subgait.duration, new_duration)
        self.assertEqual([joint.duration for joint in self.subgait.joints], [new_duration] * 8)

    # can_mirror test
    def test_can_mirror(self):
        self.assertTrue(self.subgait.can_mirror('left', 'right'))

    # get_mirror tests
    def test_get_mirror_subgait_name(self):
        mirrored_subgait = self.subgait.get_mirror('left', 'right')
        self.assertEqual(mirrored_subgait.subgait_name, self.subgait_name.replace('left', 'right'))

    def test_get_mirror_joint_trajectories(self):
        mirrored_subgait = self.subgait.get_mirror('left', 'right')
        for joint_type in ['_knee', '_hip_aa', '_hip_fe', '_ankle']:
            self.assertEqual(mirrored_subgait.get_joint('left' + joint_type).setpoints,
                             self.subgait.get_joint('right' + joint_type).setpoints,
                             msg='mirrored joint_trajectory left{type} does not have the same setpoints as '
                                 'original joint_trajectory right{type}'.format(type=joint_type))
            self.assertEqual(mirrored_subgait.get_joint('right' + joint_type).setpoints,
                             self.subgait.get_joint('left' + joint_type).setpoints,
                             msg='mirrored joint_trajectory right{type} does not have the same setpoints as '
                                 'original joint_trajectory left{type}'.format(type=joint_type))
