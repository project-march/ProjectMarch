#!/usr/bin/env python

import unittest

import rospy

from march_shared_resources.srv import GetParamBool, GetParamBoolRequest, GetParamFloat, GetParamFloatRequest, \
    GetParamInt, GetParamIntRequest, GetParamString, GetParamStringList, GetParamStringListRequest, \
    GetParamStringRequest, SetParamBool, SetParamBoolRequest, SetParamFloat, SetParamFloatRequest, SetParamInt, \
    SetParamIntRequest, SetParamString, SetParamStringList, SetParamStringListRequest, SetParamStringRequest


PKG = 'march_parameter_server'
NAME = 'test_parameter_server'


class TestParameterServer(unittest.TestCase):
    def test_get_parameter_string(self):
        rospy.set_param('/test/string_get', 'test_string123')

        rospy.wait_for_service('/march/parameter_server/get_param_string')

        service = rospy.ServiceProxy('/march/parameter_server/get_param_string', GetParamString)
        response = service.call(GetParamStringRequest('/test/string_get'))

        self.assertEqual(response.value, 'test_string123')
        self.assertTrue(response.success)

    def test_set_parameter_string(self):
        rospy.wait_for_service('/march/parameter_server/set_param_string')

        service = rospy.ServiceProxy('/march/parameter_server/set_param_string', SetParamString)
        response = service.call(SetParamStringRequest('/test/string_set', 'test_string2'))

        self.assertEqual(rospy.get_param('/test/string_set'), 'test_string2')
        self.assertTrue(response.success)

    def test_get_parameter_string_not_existing(self):
        rospy.wait_for_service('/march/parameter_server/get_param_string')

        service = rospy.ServiceProxy('/march/parameter_server/get_param_string', GetParamString)
        response = service.call(GetParamStringRequest('/not_existing_param_string'))

        self.assertEqual(response.value, '')
        self.assertFalse(response.success)

    def test_get_parameter_int(self):
        rospy.set_param('/test/int_get', 9)

        rospy.wait_for_service('/march/parameter_server/get_param_int')

        service = rospy.ServiceProxy('/march/parameter_server/get_param_int', GetParamInt)
        response = service.call(GetParamIntRequest('/test/int_get'))

        self.assertEqual(response.value, 9)
        self.assertTrue(response.success)

    def test_set_parameter_int(self):
        rospy.wait_for_service('/march/parameter_server/set_param_int')

        service = rospy.ServiceProxy('/march/parameter_server/set_param_int', SetParamInt)
        response = service.call(SetParamIntRequest('/test/int_set', -3))

        self.assertEqual(rospy.get_param('/test/int_set'), -3)
        self.assertTrue(response.success)

    def test_get_parameter_int_not_existing(self):
        rospy.wait_for_service('/march/parameter_server/get_param_int')

        service = rospy.ServiceProxy('/march/parameter_server/get_param_int', GetParamInt)
        response = service.call(GetParamIntRequest('/not_existing_param_int'))

        self.assertEqual(response.value, 0)
        self.assertFalse(response.success)

    def test_get_parameter_float(self):
        rospy.set_param('/test/float_get', 4.5)

        rospy.wait_for_service('/march/parameter_server/get_param_float')

        service = rospy.ServiceProxy('/march/parameter_server/get_param_float', GetParamFloat)
        response = service.call(GetParamFloatRequest('/test/float_get'))

        self.assertEqual(response.value, 4.5)
        self.assertTrue(response.success)

    def test_set_parameter_float(self):
        rospy.wait_for_service('/march/parameter_server/set_param_float')

        service = rospy.ServiceProxy('/march/parameter_server/set_param_float', SetParamFloat)
        response = service.call(SetParamFloatRequest('/test/float_set', -7.2))

        self.assertEqual(rospy.get_param('/test/float_set'), -7.2)
        self.assertTrue(response.success)

    def test_get_parameter_float_not_existing(self):
        rospy.wait_for_service('/march/parameter_server/get_param_float')

        service = rospy.ServiceProxy('/march/parameter_server/get_param_float', GetParamFloat)
        response = service.call(GetParamFloatRequest('/not_existing_param_float'))

        self.assertEqual(response.value, 0.0)
        self.assertFalse(response.success)

    def test_get_parameter_bool(self):
        rospy.set_param('/test/bool_get', True)

        rospy.wait_for_service('/march/parameter_server/get_param_bool')

        service = rospy.ServiceProxy('/march/parameter_server/get_param_bool', GetParamBool)
        response = service.call(GetParamBoolRequest('/test/bool_get'))

        self.assertEqual(response.value, True)
        self.assertTrue(response.success)

    def test_set_parameter_bool(self):
        rospy.wait_for_service('/march/parameter_server/set_param_bool')

        service = rospy.ServiceProxy('/march/parameter_server/set_param_bool', SetParamBool)
        response = service.call(SetParamBoolRequest('/test/bool_set', False))

        self.assertEqual(rospy.get_param('/test/bool_set'), False)
        self.assertTrue(response.success)

    def test_get_parameter_bool_not_existing(self):
        rospy.wait_for_service('/march/parameter_server/get_param_bool')

        service = rospy.ServiceProxy('/march/parameter_server/get_param_bool', GetParamBool)
        response = service.call(GetParamBoolRequest('/not_existing_param_bool'))

        self.assertEqual(response.value, False)
        self.assertFalse(response.success)

    def test_get_parameter_string_list(self):
        rospy.set_param('/test/string_list_get', ['abcdef'])

        rospy.wait_for_service('/march/parameter_server/get_param_string_list')

        service = rospy.ServiceProxy('/march/parameter_server/get_param_string_list', GetParamStringList)
        response = service.call(GetParamStringListRequest('/test/string_list_get'))

        self.assertEqual(response.value, ['abcdef'])
        self.assertTrue(response.success)

    def test_set_parameter_string_list(self):
        rospy.wait_for_service('/march/parameter_server/set_param_string_list')

        service = rospy.ServiceProxy('/march/parameter_server/set_param_string_list', SetParamStringList)
        response = service.call(SetParamStringListRequest('/test/string_list_set', ['bla', 'bloo']))

        self.assertEqual(rospy.get_param('/test/string_list_set'), ['bla', 'bloo'])
        self.assertTrue(response.success)

    def test_get_parameter_string_list_not_existing(self):
        rospy.wait_for_service('/march/parameter_server/get_param_string_list')

        service = rospy.ServiceProxy('/march/parameter_server/get_param_string_list', GetParamStringList)
        response = service.call(GetParamStringListRequest('/not_existing_param_string_list'))

        self.assertEqual(response.value, [])
        self.assertFalse(response.success)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestParameterServer)
