/*Authors: Andrew Hutani, MIX

This node is a seperate IPD for the testing of single joints. It only uses modes BootUp, Stand, and Walk, where BootUp is used to switch joints and 
Walk to send a sinusoidal wave to the joint.

This node is only called in the test_joints launch file.

*/

#include "march_input_device/test_joints_input_device_node.hpp"
#include "march_input_device/test_joints_input_device.hpp"
#include <cstdlib>
#include <array>
#include <algorithm>
using std::placeholders::_1;

std::array<ExoMode, 3> all_possible_modes = { ExoMode::Stand, ExoMode::Walk, ExoMode::BootUp};
//TODO: Re-enable these joints once the issue with hip_aa's is fixed.
// std::vector<std::string> all_possible_joints = {"left_hip_aa", "left_hip_fe", "left_knee", "left_ankle", 
//                                             "right_hip_aa", "right_hip_fe", "right_knee", "right_ankle"};

std::vector<std::string> all_possible_joints = {"left_hip_fe", "left_knee", "left_ankle", 
                                            "right_hip_fe", "right_knee", "right_ankle"};

TestJointsInputDeviceNode::TestJointsInputDeviceNode()
    : Node("march_input_device_node"),
    m_ipd (TestJointsIPD())
{
    m_get_exo_mode_array_client = 
        create_client<march_shared_msgs::srv::GetExoModeArray>("get_exo_mode_array");
    while (!m_get_exo_mode_array_client->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_WARN(this->get_logger(), "Waiting for service get_exo_mode_array to become available...");
    }
    
    RCLCPP_INFO(this->get_logger(), "Connected to service get_exo_mode_array");
    m_request = std::make_shared<march_shared_msgs::srv::GetExoModeArray::Request>();

    m_request->actuated_joint.data = askJoint();    
    m_ipd.setCurrentMode(ExoMode::BootUp);
    sendNewModeAndJoint(m_ipd.getCurrentMode());
}

void TestJointsInputDeviceNode::sendNewModeAndJoint(const ExoMode& desired_mode)
{
    m_request->desired_mode.mode = static_cast<int32_t>(desired_mode);

    auto result_future = m_get_exo_mode_array_client->async_send_request(m_request);
    
    // Wait for the result
    if (rclcpp::spin_until_future_complete(get_node_base_interface(), result_future) ==
        rclcpp::executor::FutureReturnCode::SUCCESS)
    {
        auto result = result_future.get();
        // Do something with the result
        std::set<ExoMode> exo_modes_set;
        for (const auto& exo_mode_msg : result->mode_array.modes) {
            exo_modes_set.insert(static_cast<ExoMode>(exo_mode_msg.mode));
        }
        m_ipd.setAvailableModes(exo_modes_set);
        m_ipd.setCurrentMode((ExoMode)m_request->desired_mode.mode); 

        ExoMode desired_mode = askMode();
        if (desired_mode == ExoMode::BootUp){
            m_ipd.setActuatedJoint(askJoint());
            m_request->actuated_joint.data = m_ipd.getActuatedJoint();
        }
        sendNewModeAndJoint(desired_mode);
    }
    else {
        RCLCPP_ERROR(this->get_logger(), "Failed to call service GetExoModeArray");
    }
}

ExoMode TestJointsInputDeviceNode::askMode() const
{
    std::set<ExoMode> available_modes = m_ipd.getAvailableModes();

    std::map<std::string, ExoMode> mode_map;
    for (const auto& mode : all_possible_modes) {
        mode_map[toString(mode)] = mode;
    }

    while (rclcpp::ok()){
        std::cout << "Available modes are: ";
        for (const auto& mode : available_modes) {
        std::cout << toString(mode) << " ";
        }
        std::cout<<std::endl;
        std::cout << "Enter desired mode: ";

        std::string userInput;
        std::cin >> userInput;
        
        auto it = mode_map.find(userInput);
        if (it != mode_map.end()) {
            return it->second;
        } else {
            std::cout << "Invalid mode. Please enter a valid mode." << std::endl;
        }
    }
    rclcpp::shutdown();
    return ExoMode::BootUp;
}

std::string TestJointsInputDeviceNode::askJoint() const
{
    while (rclcpp::ok()){
        std::cout << "Available joints are: ";
        for (const auto& joint : all_possible_joints) {
        std::cout << joint << " ";
        }
        std::cout<<std::endl;
        std::cout << "Enter desired joint: ";

        std::string userInput;
        std::cin >> userInput;
        
        auto it = std::find(all_possible_joints.begin(), all_possible_joints.end(), userInput);
        if (it != all_possible_joints.end()) {
            return *it;
        } else {
            std::cout << "Invalid joint. Please enter a valid joint." << std::endl;
        }
    }
    rclcpp::shutdown();
    return "left_hip_aa";
}

int main(int argc, char *argv[]) 
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<TestJointsInputDeviceNode>());
  rclcpp::shutdown();
  return 0;
}

