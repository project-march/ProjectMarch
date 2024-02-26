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

#ifndef MARCH_FUZZY_WEIGHTS_CONTROLLER__FUZZY_WEIGHTS_CONTROLLER_HPP_
#define MARCH_FUZZY_WEIGHTS_CONTROLLER__FUZZY_WEIGHTS_CONTROLLER_HPP_

#include <string>

#include "forward_command_controller/forward_command_controller.hpp"
#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/handle.hpp"
#include "fuzzy_weights_controller/visibility_control.h"
#include "realtime_tools/realtime_publisher.h"

namespace hardware_interface
{
        constexpr char HW_IF_TORQUE_WEIGHT[] = "torque_weight";
}

namespace march_fuzzy_weights_controller
{
class FuzzyWeightsController : public forward_command_controller::ForwardCommandController
{
public:
    FUZZY_WEIGHTS_CONTROLLER_PUBLIC
    FuzzyWeightsController();

    FUZZY_WEIGHTS_CONTROLLER_PUBLIC controller_interface::return_type init(
        const std::string & controller_name) override;
};

}  // namespace march_fuzzy_weights_controller

#endif  // MARCH_FUZZY_WEIGHTS_CONTROLLER__FUZZY_WEIGHTS_CONTROLLER_HPP_