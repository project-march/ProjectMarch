#ifndef MARCH_STATE_ESTIMATOR__URDF_PARSER_HPP_
#define MARCH_STATE_ESTIMATOR__URDF_PARSER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <functional>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "urdf/model.h"

class UrdfParser
{
public:
    UrdfParser();

    void parse(const std::string & urdf_path);

private:
    urdf::Model urdf_model_;
};

#endif // MARCH_STATE_ESTIMATOR__URDF_PARSER_HPP_