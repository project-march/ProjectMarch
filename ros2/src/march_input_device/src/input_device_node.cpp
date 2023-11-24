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

  sendNewState(exoState::Stand);

}

inputDeviceNode::~inputDeviceNode()
{
  // system("pkill gnome-terminal");
  endwin();
}

void inputDeviceNode::availableStatesCallback(const march_shared_msgs::msg::ExoStateArray::SharedPtr msg)
{
  printw("Received available states: %d", msg->states);
  refresh();
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


int main(int argc, char *argv[]) 
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<inputDeviceNode>());
  rclcpp::shutdown();
  return 0;
}

