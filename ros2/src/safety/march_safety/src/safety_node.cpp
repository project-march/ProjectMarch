// Copyrihgt 2018 Project March
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"

#include "march_safety/safety_node.hpp"
#include "march_safety/safety_handler.hpp"
#include "march_safety/safety_type.hpp"
#include "march_safety/input_device_safety.hpp"
#include "march_safety/temperature_safety.hpp"

#include "march_shared_msgs/srv/get_param_string_list.hpp"
#include "march_shared_msgs/msg/error.hpp"
#include "march_shared_msgs/msg/gait_instruction.hpp"

#include "march_shared_functions/march_util.hpp"

#include <chrono>

const double UPDATE_RATE {20.0};

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto safety = std::make_shared<SafetyNode>("march_safety", options);

  safety->start(UPDATE_RATE);

  rclcpp::shutdown();
  return 0;
}

SafetyNode::SafetyNode(const std::string& node_name, const rclcpp::NodeOptions& options):
  Node(node_name, options)
{
  std::vector<std::string> joint_names = march_util::get_joint_names(*this);

  RCLCPP_DEBUG(this->get_logger(), "Got joint names.");

  // Create an error publisher to notify the system (state machine) if something is wrong
  auto error_publisher = this->create_publisher<march_shared_msgs::msg::Error>("/march/error", 1000);
  auto gait_instruction_publisher = this->create_publisher<march_shared_msgs::msg::GaitInstruction>("/march/input_device/instruction", 1000);

  // Create the input and temperature safety handler
  auto safety_handler = std::make_shared<SafetyHandler>(this, error_publisher, gait_instruction_publisher);
  safety_list.push_back(std::make_unique<TemperatureSafety>(this, safety_handler, joint_names));
  safety_list.push_back(std::make_unique<InputDeviceSafety>(this, safety_handler));
}

void SafetyNode::start(const double update_rate)
{
  // Update the safety handlers every 1/20 s (= 50ms)
  rclcpp::Rate rate(update_rate);
  while (rclcpp::ok())
  {
    rate.sleep();
    rclcpp::spin_some(this->get_node_base_interface());

    for (auto& i : safety_list)
    {
      i->update();
    }
  }
}