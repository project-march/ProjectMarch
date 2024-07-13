#include "march_gait_planning/listener_gait_planning.hpp"

using std::placeholders::_1; 

NodeManagerGaitPlanning::NodeManagerGaitPlanning()
 : Node("listener_gait_planning")
 {

    m_angles_notification_subscriber = this->create_subscription<lifecycle_msgs::msg::TransitionEvent>("gait_planning_angles_node/transition_event", 10, std::bind(&NodeManagerGaitPlanning::anglesNotificationCallback, this, _1));
    m_cartesian_notification_subscriber = this->create_subscription<lifecycle_msgs::msg::TransitionEvent>("gait_planning_cartesian_node/transition_event", 10, std::bind(&NodeManagerGaitPlanning::cartesianNotificationCallback, this, _1));

 }


void NodeManagerGaitPlanning::anglesNotificationCallback(const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg){
   if (msg->goal_state.id == 3 || msg->goal_state.id == 2){
    RCLCPP_DEBUG(this->get_logger(), "Joint angle node is in state %s ", msg->goal_state.label.c_str()); 
   }
   //  RCLCPP_INFO(this->get_logger(), "notificationCallback: transition Joint angle node from state %s to %s \n", msg->start_state.label.c_str(), msg->goal_state.label.c_str()); 
 }

 void NodeManagerGaitPlanning::cartesianNotificationCallback(const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg){
   if (msg->goal_state.id == 3 || msg->goal_state.id == 2){
    RCLCPP_DEBUG(this->get_logger(), "Cartesian node is in state %s ", msg->goal_state.label.c_str()); 
   }
   //  RCLCPP_INFO(this->get_logger(), "notificationCallback: transition Cartesian node from state %s to %s \n", msg->start_state.label.c_str(), msg->goal_state.label.c_str()); 
 }

int main(int argc, char *argv[]){
    
    rclcpp::init(argc, argv); 

    rclcpp::spin(std::make_shared<NodeManagerGaitPlanning>()); 
    rclcpp::shutdown(); 

    return 0; 
}