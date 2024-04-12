#include "march_gait_planning/listener_gait_planning.hpp"

using std::placeholders::_1; 

NodeManagerGaitPlanning::NodeManagerGaitPlanning()
 : Node("listener_gait_planning")
 {
    //FILMPJE
    m_message_subscriber = this->create_subscription<std_msgs::msg::String>("messages", 10, std::bind(&NodeManagerGaitPlanning::messageCallback, this, _1));
    m_notification_subscriber = this->create_subscription<lifecycle_msgs::msg::TransitionEvent>("gait_planning_angles_node/transition_event", 10, std::bind(&NodeManagerGaitPlanning::notificationCallback, this, _1));
    //

    m_mode_subscriber = create_subscription<march_shared_msgs::msg::ExoMode>(
        "current_mode", 10, std::bind(&NodeManagerGaitPlanning::activationCallback, this, _1)); 
    // m_angles_node = std::make_shared<GaitPlanningAnglesNode>(); 
    // m_angles_node->configure(); 
 }

 //FILMPJE
void NodeManagerGaitPlanning::messageCallback(const std_msgs::msg::String::SharedPtr msg){
    RCLCPP_INFO(this->get_logger(), "messageCallback: %s", msg->data.c_str()); 
 }

void NodeManagerGaitPlanning::notificationCallback(const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg){
    RCLCPP_INFO(this->get_logger(), "notificationCallback: transition from state %s to %s", msg->start_state.label.c_str(), msg->goal_state.label.c_str()); 
 }
 //

void NodeManagerGaitPlanning::activationCallback(const march_shared_msgs::msg::ExoMode::SharedPtr msg){
    if (msg->node_type == "joint_angles" && msg->mode == 1){
        activateAnglesNode(msg); 
    } else {
        RCLCPP_WARN(this->get_logger(), "Unknown node type: %s", msg->node_type.c_str()); 
    }
}

void NodeManagerGaitPlanning::activateAnglesNode(const march_shared_msgs::msg::ExoMode::SharedPtr msg){
    // auto node = std::make_shared<GaitPlanningAnglesNode>();

    // cartesian_node->deactivate(); 

    // angles_node->configure();
    // m_angles_node->setFirstCallbackMsg(msg);  
    // m_angles_node->activate(); 
    // m_angles_node->currentModeCallback(msg); 
}

void NodeManagerGaitPlanning::deactivateAnglesNode(){
    m_angles_node->deactivate(); 
}

int main(int argc, char *argv[]){
    
    rclcpp::init(argc, argv); 

    rclcpp::spin(std::make_shared<NodeManagerGaitPlanning>()); 
    rclcpp::shutdown(); 

    return 0; 
}