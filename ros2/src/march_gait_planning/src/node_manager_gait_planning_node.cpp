#include "march_gait_planning/node_manager_gait_planning_node.hpp"

using std::placeholders::_1; 

NodeManagerGaitPlanning::NodeManagerGaitPlanning()
 : Node("gait_planning_manager")
 {
    m_mode_subscriber = create_subscription<march_shared_msgs::msg::ExoMode>(
        "current_mode", 10, std::bind(&NodeManagerGaitPlanning::activationCallback, this, _1)); 
 }

void NodeManagerGaitPlanning::activationCallback(const march_shared_msgs::msg::ExoMode::SharedPtr msg){
    if (msg->node_type == "joint_angles"){
        activateGaitPlanningAnglesNode(); 
    // } else if (msg->node_type == "cartesian"){
    //     activateGaitPlanningCartesianNode();
    // } else if (msg->node_type == "test_joints"){
    //     activateGaitPlanningTestJointsNode(); 
    // } else if (msg->node_type == "test_setup"){
    //     activateGaitPlanningTestSetupNode(); 
    } else {
        RCLCPP_WARN(this->get_logger(), "Unknown node type: %s", msg->node_type.c_str()); 
    }
}

void NodeManagerGaitPlanning::activateGaitPlanningAnglesNode(){
    auto node = std::make_shared<GaitPlanningAnglesNode>(); 
    node->configure(); 
    node->activate(); 
}

int main(int argc, char *argv[]){
    
    rclcpp::init(argc, argv); 

    rclcpp::spin(std::make_shared<NodeManagerGaitPlanning>()); 
    rclcpp::shutdown(); 

    return 0; 
}