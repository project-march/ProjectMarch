//
// Created by andrew on 23-11-23.
//

#include "march_input_device/input_device_node.hpp"
#include "march_input_device/input_device.hpp"
#include <cstdlib>
#include <ncurses.h>
using std::placeholders::_1;

inputDeviceNode::inputDeviceNode()
  : Node("march_input_device_node"),
  m_ipd (IPD())
{
  m_new_state_publisher = create_publisher<std_msgs::msg::Int32>("new_state", 10);
  m_exo_state_array_subscriber = create_subscription<march_shared_msgs::msg::ExoStateArray>(
    "available_states", 10, std::bind(&inputDeviceNode::availableStatesCallback, this, _1));

  // system("gnome-terminal &");

  // Initialize ncurses
  initscr();
  cbreak();
  noecho();

  sendNewState(m_ipd.getCurrentState());
  m_ipd.setCurrentState(exoState::Stand);

}

inputDeviceNode::~inputDeviceNode()
{
  // system("pkill gnome-terminal");
  endwin();
}

void inputDeviceNode::availableStatesCallback(const march_shared_msgs::msg::ExoStateArray::SharedPtr msg)
{
  // Iterate over the states vector and print each state
  for (const auto& state : msg->states) {
    printw("Received available state: %d \n", state.state);
  }
  refresh();

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
  printw("Sent new state: %d\n", msg.data);

  // Refresh the screen
  refresh();
}

exoState inputDeviceNode::askState() const
{
  std::set<exoState> available_states = m_ipd.getAvailableStates();

  std::map<std::string, exoState> state_map;
  for (const auto& state : available_states) {
    state_map[toString(state)] = state;
  }

  while (true){
    printw("Available states are: ");
    for (const auto& state : available_states) {
      printw("%s, ", toString(state).c_str());
    }
    printw("\n Please input new state:");
    refresh();

    // Get the user input
    char str[100];
    getstr(str);
    std::string input(str);

    return state_map[input];
    
  }
}


int main(int argc, char *argv[]) 
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<inputDeviceNode>());
  rclcpp::shutdown();
  return 0;
}

