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

#ifndef MARCH_SCHEDULED_GAINS_CONTROLLER__SCHEDULED_GAINS_CONTROLLER_HPP_
#define MARCH_SCHEDULED_GAINS_CONTROLLER__SCHEDULED_GAINS_CONTROLLER_HPP_

#include <string>

#include "forward_command_controller/forward_command_controller.hpp"
#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/handle.hpp"
#include "scheduled_gains_controller/visibility_control.h"
#include "realtime_tools/realtime_publisher.h"

namespace hardware_interface
{
    constexpr char HW_IF_PROPORTIONAL_GAIN[] = "proportional_gain";
    constexpr char HW_IF_DERIVATIVE_GAIN[] = "derivative_gain";
    constexpr char HW_IF_INTEGRAL_GAIN[] = "integral_gain";
}

namespace march_scheduled_gains_controller
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using CmdType = std_msgs::msg::Float64MultiArray;
class ScheduledGainsController : public forward_command_controller::ForwardCommandController
{
public:
  SCHEDULED_GAINS_CONTROLLER_PUBLIC
  ScheduledGainsController();

  SCHEDULED_GAINS_CONTROLLER_PUBLIC 
  controller_interface::return_type init(const std::string & controller_name) override;

  SCHEDULED_GAINS_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  SCHEDULED_GAINS_CONTROLLER_PUBLIC 
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;


private:
  std::vector<std::string> m_joint_names;
  std::vector<std::string> m_interface_names;
};

}  // namespace march_scheduled_gains_controller

#endif  // MARCH_SCHEDULED_GAINS_CONTROLLER__SCHEDULED_GAINS_CONTROLLER_HPP_
