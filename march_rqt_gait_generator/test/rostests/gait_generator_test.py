import unittest
import rospy

import numpy as np
import time

from march_rqt_gait_generator.gait_generator import GaitGeneratorPlugin
import threading

PKG = 'march_rqt_gait_generator'

class GaitGeneratorPluginTest(unittest.TestCase):
    def setUp(self):
        pass

    def test_something(self):
        rospy.init_node("gait_generator_tests")
        self.gait_generator = GaitGeneratorPlugin(None)
        self.assertTrue(False)
