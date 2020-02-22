import unittest

from mock import Mock
import rospkg
from urdf_parser_py import urdf

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
    def test_has_multiple_setpoints








