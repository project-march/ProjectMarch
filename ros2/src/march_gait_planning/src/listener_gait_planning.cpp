#include "march_gait_planning/listener_gait_planning.hpp"

using std::placeholders::_1; 

NodeManagerGaitPlanning::NodeManagerGaitPlanning()
 : Node("listener_gait_planning")
 {

    m_angles_notification_subscriber = this->create_subscription<lifecycle_msgs::msg::TransitionEvent>("gait_planning_angles_node/transition_event", 10, std::bind(&NodeManagerGaitPlanning::anglesNotificationCallback, this, _1));
    m_cartesian_notification_subscriber = this->create_subscription<lifecycle_msgs::msg::TransitionEvent>("gait_planning_cartesian_node/transition_event", 10, std::bind(&NodeManagerGaitPlanning::cartesianNotificationCallback, this, _1));

    // m_angles_state_subscriber = this->create_subscription<lifecycle_msgs::srv::GetState>("gait_planning_angles_node/get_state", 10, std::bind(&NodeManagerGaitPlanning::anglesStateCallback, this, _1));
    // m_cartesian_state_subscriber = this->create_subscription<lifecycle_msgs::srv::GetState>("gait_planning_cartesian_node/get_state", 10, std::bind(&NodeManagerGaitPlanning::cartesianStateCallback, this, _1));

 }



void NodeManagerGaitPlanning::anglesNotificationCallback(const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg){
    RCLCPP_INFO(this->get_logger(), "notificationCallback: transition Join angle node from state %s to %s \n", msg->start_state.label.c_str(), msg->goal_state.label.c_str()); 
 }

 void NodeManagerGaitPlanning::cartesianNotificationCallback(const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg){
    RCLCPP_INFO(this->get_logger(), "notificationCallback: transition Cartesian node from state %s to %s \n", msg->start_state.label.c_str(), msg->goal_state.label.c_str()); 
 }

//  void NodeManagerGaitPlanning::anglesStateCallback(const lifecycle_msgs::srv::GetState::SharedPtr msg){
//     RCLCPP_INFO(this->get_logger(), "stateCallback: get JA state service %s", msg->current_state); 
//  }

//  void NodeManagerGaitPlanning::cartesianStateCallback(const lifecycle_msgs::srv::GetState::SharedPtr msg){
//     RCLCPP_INFO(this->get_logger(), "stateCallback: get cartesian state service %s", msg->current_state); 
//  }

int main(int argc, char *argv[]){
    
    rclcpp::init(argc, argv); 

    rclcpp::spin(std::make_shared<NodeManagerGaitPlanning>()); 
    rclcpp::shutdown(); 

    return 0; 
}