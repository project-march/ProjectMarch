import unittest

from mock import Mock
import rospkg
from urdf_parser_py import urdf

from march_rqt_gait_generator.model.modifiable_subgait import ModifiableSubgait


class ModifiableSubgaitTest(unittest.TestCase):
    def setUp(self):
        self.gait_generator = Mock({'save_changed_joints': None})
        self.gait_name = 'walk'
        self.subgait_name = 'left_swing'
        self.version = 'MV_walk_leftswing_v2'
        self.resources_folder = rospkg.RosPack().get_path('march_rqt_gait_generator') + '/test/resources'
        self.robot = urdf.Robot.from_xml_file(self.resources_folder + '/march4.urdf')
        self.subgait_path = '{rsc}/{gait}/{subgait}/{version}.subgait'.format(rsc=self.resources_folder,
                                                                              gait=self.gait_name,
                                                                              subgait=self.subgait_name,
                                                                              version=self.version)
        self.subgait = ModifiableSubgait.from_file(self.gait_generator, self.robot, self.subgait_path)
        self.subgait_msg = self.subgait.to_subgait_msg()

    def test_something(self):
        self.assertTrue(True)
