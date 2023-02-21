#ifndef __clang_analyzer__
// NOLINTBEGIN
#pragma once
#include "state_estimator.hpp"
#include <gmock/gmock.h>
#include <unordered_map>
struct MockParameter {
public:
    std::vector<int64_t> content_int64;
    std::vector<double> content_double;
    std::vector<std::string> content_string;

    std::vector<int64_t> as_integer_array()
    {
        return content_int64;
    };
    std::vector<double> as_double_array()
    {
        return content_double;
    };
    std::vector<std::string> as_string_array()
    {
        return content_string;
    };
};

class MockStateEstimator {
public:
    MockStateEstimator() {};

    void declare_parameter(std::string name, std::vector<std::string> type) {};
    void declare_parameter(std::string name, std::vector<int64_t> type) {};
    void declare_parameter(std::string name, std::vector<double> type) {};

    sensor_msgs::msg::JointState get_initial_joint_states()
    {
        sensor_msgs::msg::JointState initial_joint_state;
        // change it so the names are obtained from the parameter
        initial_joint_state.name = { "right_ankle", "right_knee", "right_hip_fe", "right_hip_aa", "left_ankle",
            "left_knee", "left_hip_fe", "left_hip_aa", "right_origin", "left_origin" };
        initial_joint_state.position = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
        return initial_joint_state;
    };

    void add_mock_parameter(std::string name)
    {
        m_parameter_list.insert(std::pair<std::string, MockParameter> { name, MockParameter() });
    };

    std::unordered_map<std::string, MockParameter> m_parameter_list;

    MockParameter get_parameter(std::string name)
    {
        std::unordered_map<std::string, MockParameter>::iterator it;
        it = m_parameter_list.find(name);
        if (it != m_parameter_list.end()) {
            return it->second;
        } else {
            return MockParameter();
        }
    };
};
// NOLINTEND
#endif
