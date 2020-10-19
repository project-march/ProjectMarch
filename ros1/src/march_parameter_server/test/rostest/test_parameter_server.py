#!/usr/bin/env python

import unittest

import rospy

from march_shared_resources.srv import GetParamInt, GetParamIntRequest, GetParamString, GetParamStringRequest, \
    SetParamString, SetParamStringRequest


PKG = 'march_parameter_server'
NAME = 'test_parameter_server'


class TestParameterServer(unittest.TestCase):
    def test_get_parameter(self):
        rospy.set_param('/test', 'test_string123')

        rospy.wait_for_service('/march/parameter_server/get_param_string')

        service = rospy.ServiceProxy('/march/parameter_server/get_param_string', GetParamString)
        response = service.call(GetParamStringRequest('/test'))

        self.assertEqual(response.value, 'test_string123')
        self.assertTrue(response.success)

    def test_set_parameter(self):
        rospy.wait_for_service('/march/parameter_server/set_param_string')

        service = rospy.ServiceProxy('/march/parameter_server/set_param_string', SetParamString)
        response = service.call(SetParamStringRequest('/test2', 'test_string2'))

        self.assertEqual(rospy.get_param('/test2'), 'test_string2')
        self.assertTrue(response.success)

    def test_get_parameter_not_existing(self):
        rospy.wait_for_service('/march/parameter_server/get_param_int')

        service = rospy.ServiceProxy('/march/parameter_server/get_param_int', GetParamInt)
        response = service.call(GetParamIntRequest('/not_existing_param'))

        self.assertEqual(response.value, 0)
        self.assertFalse(response.success)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestParameterServer)
