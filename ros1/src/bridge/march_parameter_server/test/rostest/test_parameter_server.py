#!/usr/bin/env python

import unittest

from parameterized import parameterized
import rospy

from march_shared_msgs.srv import (
    GetParamBool,
    GetParamBoolRequest,
    GetParamFloat,
    GetParamFloatRequest,
    GetParamInt,
    GetParamIntRequest,
    GetParamString,
    GetParamStringList,
    GetParamStringListRequest,
    GetParamStringRequest,
    SetParamBool,
    SetParamBoolRequest,
    SetParamFloat,
    SetParamFloatRequest,
    SetParamInt,
    SetParamIntRequest,
    SetParamString,
    SetParamStringList,
    SetParamStringListRequest,
    SetParamStringRequest,
)


PKG = "march_parameter_server"
NAME = "test_parameter_server"


class TestParameterServer(unittest.TestCase):
    @parameterized.expand(
        [
            ["float", GetParamFloat, GetParamFloatRequest, 4.5],
            ["int", GetParamInt, GetParamIntRequest, 9],
            ["string", GetParamString, GetParamStringRequest, "test_string123"],
            ["bool", GetParamBool, GetParamBoolRequest, True],
            ["string_list", GetParamStringList, GetParamStringListRequest, ["abcdef"]],
        ]
    )
    def test_get_parameter(self, param_type, service_type, service_request_type, value):
        rospy.set_param("/test/{param_type}_get".format(param_type=param_type), value)

        rospy.wait_for_service(
            "/march/parameter_server/get_param_{param_type}".format(
                param_type=param_type
            )
        )

        service = rospy.ServiceProxy(
            "/march/parameter_server/get_param_{param_type}".format(
                param_type=param_type
            ),
            service_type,
        )
        response = service.call(
            service_request_type("/test/{param_type}_get".format(param_type=param_type))
        )

        self.assertEqual(response.value, value)
        self.assertTrue(response.success)

    @parameterized.expand(
        [
            ["float", SetParamFloat, SetParamFloatRequest, -7.2],
            ["int", SetParamInt, SetParamIntRequest, -3],
            ["string", SetParamString, SetParamStringRequest, "abcdef"],
            ["bool", SetParamBool, SetParamBoolRequest, False],
            [
                "string_list",
                SetParamStringList,
                SetParamStringListRequest,
                ["abcde", "fghijklm"],
            ],
        ]
    )
    def test_set_parameter(self, param_type, service_type, service_request_type, value):
        rospy.wait_for_service(
            "/march/parameter_server/set_param_{param_type}".format(
                param_type=param_type
            )
        )

        service = rospy.ServiceProxy(
            "/march/parameter_server/set_param_{param_type}".format(
                param_type=param_type
            ),
            service_type,
        )
        response = service.call(
            service_request_type(
                "/test/{param_type}_set".format(param_type=param_type), value
            )
        )

        self.assertEqual(
            rospy.get_param("/test/{param_type}_set".format(param_type=param_type)),
            value,
        )
        self.assertTrue(response.success)

    @parameterized.expand(
        [
            ["float", GetParamFloat, GetParamFloatRequest, 0.0],
            ["int", GetParamInt, GetParamIntRequest, 0],
            ["string", GetParamString, GetParamStringRequest, ""],
            ["bool", GetParamBool, GetParamBoolRequest, False],
            ["string_list", GetParamStringList, GetParamStringListRequest, []],
        ]
    )
    def test_get_parameter_not_existing(
        self, param_type, service_type, service_request_type, default_value
    ):
        rospy.wait_for_service(
            "/march/parameter_server/get_param_{param_type}".format(
                param_type=param_type
            )
        )

        service = rospy.ServiceProxy(
            "/march/parameter_server/get_param_{param_type}".format(
                param_type=param_type
            ),
            service_type,
        )
        response = service.call(
            service_request_type(
                "/not_existing_param_{param_type}".format(param_type=param_type)
            )
        )

        self.assertEqual(response.value, default_value)
        self.assertFalse(response.success)


if __name__ == "__main__":
    import rostest

    rostest.rosrun(PKG, NAME, TestParameterServer)
