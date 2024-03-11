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
#include "scheduled_gains_controller/scheduled_gains_controller.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/parameter.hpp"

namespace march_scheduled_gains_controller
{

ScheduledGainsController::ScheduledGainsController(): forward_command_controller::ForwardCommandController()
{
  logger_name_ = "Scheduled Gains Controller";
  m_interface_names = {hardware_interface::HW_IF_PROPORTIONAL_GAIN,
                       hardware_interface::HW_IF_DERIVATIVE_GAIN,
                       hardware_interface::HW_IF_INTEGRAL_GAIN};
}

controller_interface::return_type ScheduledGainsController::init(const std::string & controller_name)
{
  auto ret = ForwardCommandController::init(controller_name);
  if (ret != controller_interface::return_type::OK)
  {
    return ret;
  }

  try
  {
    // Explicitly set the interface parameter declared by the forward_command_controller
    // get_node()->set_parameter(
    //   rclcpp::Parameter("interface_name", hardware_interface::HW_IF_PROPORTIONAL_GAIN));
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}


CallbackReturn ScheduledGainsController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  m_joint_names = node_->get_parameter("joints").as_string_array();
  m_interface_names = node_->get_parameter("command_interfaces").as_string_array();

  if (m_joint_names.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter was empty");
    return CallbackReturn::ERROR;
  }

  if (m_interface_names.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'interface_names' parameter was empty");
    return CallbackReturn::ERROR;
  }

  joints_command_subscriber_ = get_node()->create_subscription<CmdType>(
    "~/commands", rclcpp::SystemDefaultsQoS(),
    [this](const CmdType::SharedPtr msg) { rt_command_ptr_.writeFromNonRT(msg); });

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration ScheduledGainsController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & joint : m_joint_names)
  {
    for (const auto & interface_name : m_interface_names){

      command_interfaces_config.names.push_back(joint + "/" + interface_name);

    }
    
  }

  return command_interfaces_config;
}

template <typename T> bool get_ordered_interfaces(
  std::vector<T> & unordered_interfaces, const std::vector<std::string> & joint_names,
  const std::string & interface_type, std::vector<std::reference_wrapper<T>> & ordered_interfaces)
{
  ordered_interfaces.clear();
  for (const auto & joint_name : joint_names)
  {
    bool interface_found = false;
    for (auto & command_interface : unordered_interfaces)
    {
      if (
        (command_interface.get_name() == joint_name) &&
        (command_interface.get_interface_name() == interface_type))
      {
        ordered_interfaces.push_back(std::ref(command_interface));
        interface_found = true;
        break;
      }
    }
    if (!interface_found)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Interface not found for joint: %s", joint_name.c_str());
      return false;
    }
  }

  return true;
}


}  // namespace march_scheduled_gains_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  march_scheduled_gains_controller::ScheduledGainsController, controller_interface::ControllerInterface)
