/*Authors: Pleuntje Brons and Andrew Hutani, MIX*/

#include "march_input_device/input_device_node.hpp"
#include "march_input_device/input_device.hpp"
#include <cstdlib>
using std::placeholders::_1;


std::vector<exoMode> all_possible_modes;

inputDeviceNode::inputDeviceNode()
  : Node("march_input_device_node"),
  m_ipd (IPD())
{
  m_get_exo_mode_array_client = 
    create_client<march_shared_msgs::srv::GetExoModeArray>("get_exo_mode_array");
  while (!m_get_exo_mode_array_client->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_WARN(this->get_logger(), "Waiting for service get_exo_mode_array to become available...");
  }

  RCLCPP_INFO(this->get_logger(), "Connected to service get_exo_mode_array");

  m_ipd.setCurrentMode(exoMode::BootUp);
  sendNewMode(m_ipd.getCurrentMode());
  

}

void inputDeviceNode::sendNewMode(const exoMode& desired_mode)
{
    auto request = std::make_shared<march_shared_msgs::srv::GetExoModeArray::Request>();
    request->desired_mode.mode = static_cast<int32_t>(desired_mode);

    auto result_future = m_get_exo_mode_array_client->async_send_request(request);


    // Wait for the result
    if (rclcpp::spin_until_future_complete(get_node_base_interface(), result_future) ==
        rclcpp::executor::FutureReturnCode::SUCCESS)
    {
        auto result = result_future.get();
        // Do something with the result
        std::set<exoMode> exo_modes_set;
        for (const auto& exo_mode_msg : result->mode_array.modes) {
            exo_modes_set.insert(static_cast<exoMode>(exo_mode_msg.mode));
        }
        m_ipd.setAvailableModes(exo_modes_set);
    }
    else {
        RCLCPP_ERROR(this->get_logger(), "Failed to call service GetExoModeArray");
    }
}

int main(int argc, char *argv[]) 
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<inputDeviceNode>());
  rclcpp::shutdown();
  return 0;
}

