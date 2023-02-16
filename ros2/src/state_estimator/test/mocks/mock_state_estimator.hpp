#ifndef __clang_analyzer__
// NOLINTBEGIN
#pragma once
#include <gmock/gmock.h>
#include <unordered_map>

struct MockParameter {
public:
    std::vector<int64_t> content_int64;
    std::vector<double> content_double;

    std::vector<int64_t> as_integer_array()
    {
        return content_int64;
    };
    std::vector<double> as_double_array()
    {
        return content_double;
    };
};

class MockStateEstimator {
public:
    MockStateEstimator() {};

    void declare_parameter(std::string name, std::vector<int64_t> type) {};
    void declare_parameter(std::string name, std::vector<double> type) {};

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
