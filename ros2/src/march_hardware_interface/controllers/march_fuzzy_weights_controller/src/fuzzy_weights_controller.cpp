// Copyright 2020 PAL Robotics S.L.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "fuzzy_weights_controller/fuzzy_weights_controller.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/parameter.hpp"

namespace march_fuzzy_weights_controller
{

FuzzyWeightsController::FuzzyWeightsController(): forward_command_controller::ForwardCommandController()
{
    logger_name_ = "Fuzzy Weights Controller";
    interface_name_ = hardware_interface::HW_IF_TORQUE_WEIGHT;
}

controller_interface::return_type FuzzyWeightsController::init(const std::string & controller_name)
{
    auto ret = ForwardCommandController::init(controller_name);
    if (ret != controller_interface::return_type::OK)
    {
        return ret;
    }

    try
    {
        // Explicitly set the interface parameter declared by the forward_command_controller
        get_node()->set_parameter(
            rclcpp::Parameter("interface_name", hardware_interface::HW_IF_TORQUE_WEIGHT));
    }
    catch (const std::exception & e)
    {
        fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
        return controller_interface::return_type::ERROR;
    }

    return controller_interface::return_type::OK;
}
}  // namespace march_fuzzy_weights_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    march_fuzzy_weights_controller::FuzzyWeightsController, controller_interface::ControllerInterface)