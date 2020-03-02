import unittest
import rospy

import numpy as np
import time

from march_rqt_gait_generator.gait_generator_controller import GaitGeneratorController
import threading

PKG = 'march_rqt_gait_generator'

class GaitGeneratorControllerTest(unittest.TestCase):
    # def __init__(self, *args):
    #     super(GaitGeneratorControllerTest, self).__init__(*args)
    #     rospy.init_node("test_gait_generator", anonymous=True)

    def setUp(self):

        pass

    def test_something(self):
        self.assertTrue(False)

    def test_something_else(self):
        self.assertTrue(True)

if __name__ == '__main__':
    rostest.rosrun(PKG, 'gait_generator_test', GaitGeneratorController)
