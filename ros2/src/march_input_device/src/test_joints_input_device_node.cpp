/*Authors: Pleuntje Brons and Andrew Hutani, MIX*/

#include "march_input_device/test_joints_input_device_node.hpp"
#include "march_input_device/test_joints_input_device.hpp"
#include <cstdlib>
#include <array>
#include <algorithm>
using std::placeholders::_1;

std::array<exoState, 3> all_possible_states = { exoState::Stand, exoState::Walk, exoState::BootUp};
std::vector<std::string> all_possible_joints = {"left_hip_aa", "left_hip_fe", "left_knee", "left_ankle", 
                                            "right_hip_aa", "right_hip_fe", "right_knee", "right_ankle",
                                            "test_rotational", "test_linear"};

TestJointsInputDeviceNode::TestJointsInputDeviceNode()
    : Node("march_input_device_node"),
    m_ipd (TestJointsIPD())
{
    m_get_exo_state_array_client = 
        create_client<march_shared_msgs::srv::GetExoStateArray>("get_exo_state_array");
    while (!m_get_exo_state_array_client->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_WARN(this->get_logger(), "Waiting for service get_exo_state_array to become available...");
    }
    
    RCLCPP_INFO(this->get_logger(), "Connected to service get_exo_state_array");
    m_request = std::make_shared<march_shared_msgs::srv::GetExoStateArray::Request>();

    m_request->actuated_joint.data = askJoint();    
    m_ipd.setCurrentState(exoState::BootUp);
    sendNewStateAndJoint(m_ipd.getCurrentState());
}

void TestJointsInputDeviceNode::sendNewStateAndJoint(const exoState& desired_state)
{
    m_request->desired_state.state = static_cast<int32_t>(desired_state);

    auto result_future = m_get_exo_state_array_client->async_send_request(m_request);
    
    // Wait for the result
    if (rclcpp::spin_until_future_complete(get_node_base_interface(), result_future) ==
        rclcpp::executor::FutureReturnCode::SUCCESS)
    {
        auto result = result_future.get();
        // Do something with the result
        std::set<exoState> exo_states_set;
        for (const auto& exo_state_msg : result->state_array.states) {
            exo_states_set.insert(static_cast<exoState>(exo_state_msg.state));
        }
        m_ipd.setAvailableStates(exo_states_set);
        m_ipd.setCurrentState((exoState)m_request->desired_state.state); 

        exoState desired_state = askState();
        if (desired_state == exoState::BootUp){
            m_ipd.setActuatedJoint(askJoint());
            m_request->actuated_joint.data = m_ipd.getActuatedJoint();
        }
        sendNewStateAndJoint(desired_state);
    }
    else {
        RCLCPP_ERROR(this->get_logger(), "Failed to call service GetExoStateArray");
    }
}

exoState TestJointsInputDeviceNode::askState() const
{
    std::set<exoState> available_states = m_ipd.getAvailableStates();

    std::map<std::string, exoState> state_map;
    for (const auto& state : all_possible_states) {
        state_map[toString(state)] = state;
    }

    while (rclcpp::ok()){
        std::cout << "Available states are: ";
        for (const auto& state : available_states) {
        std::cout << toString(state) << " ";
        }
        std::cout<<std::endl;
        std::cout << "Enter desired state: ";

        std::string userInput;
        std::cin >> userInput;
        
        auto it = state_map.find(userInput);
        if (it != state_map.end()) {
            return it->second;
        } else {
            std::cout << "Invalid state. Please enter a valid state." << std::endl;
        }
    }
    rclcpp::shutdown();
    return exoState::BootUp;
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

