/*Authors: Pleuntje Brons and Andrew Hutani, MIX*/

#include "march_input_device/input_device_node.hpp"
#include "march_input_device/input_device.hpp"
#include <cstdlib>
#include <ncurses.h>
using std::placeholders::_1;

std::array<exoState, 6> all_possible_states = {exoState::Sit, exoState::Stand, exoState::Walk, exoState::StepClose, exoState::BootUp, exoState::Error};

inputDeviceNode::inputDeviceNode()
  : Node("march_input_device_node"),
  m_ipd (IPD())
{
  m_new_state_publisher = create_publisher<std_msgs::msg::Int32>("new_state", 10);
  m_exo_state_array_subscriber = create_subscription<march_shared_msgs::msg::ExoStateArray>(
    "available_states", 10, std::bind(&inputDeviceNode::availableStatesCallback, this, _1));

  m_ipd.setCurrentState(exoState::BootUp);
  sendNewState(m_ipd.getCurrentState());
  

}

inputDeviceNode::~inputDeviceNode()
{
}

void inputDeviceNode::availableStatesCallback(const march_shared_msgs::msg::ExoStateArray::SharedPtr msg)
{
  std::set<exoState> exo_states_set;
  for (const auto& exo_state_msg : msg->states) {
    exo_states_set.insert(static_cast<exoState>(exo_state_msg.state));
  }
  m_ipd.setAvailableStates(exo_states_set);

  exoState desired_state = askState();
  sendNewState(desired_state);
  m_ipd.setCurrentState(desired_state);

}

void inputDeviceNode::sendNewState(const exoState& desired_state)
{
  auto msg = std_msgs::msg::Int32();
  msg.data = static_cast<int32_t>(desired_state);
  m_new_state_publisher->publish(msg);

  // Print the new state
  std::cout << "Sent new state: "<< toString(exoState(msg.data)) << "\n" << std::endl;
}

exoState inputDeviceNode::askState() const
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


int main(int argc, char *argv[]) 
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<inputDeviceNode>());
  rclcpp::shutdown();
  return 0;
}

