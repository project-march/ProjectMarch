
#include "march_gait_planning/gait_planning_base_node.hpp"

using std::placeholders::_1; 

GaitPlanningNode::GaitPlanningNode()
 : Node("gait_planning_node"),
 m_gait_planning_angles_node(GaitPlanningAnglesNode()), 
 m_gait_planning_cartesian_node(GaitPlanningCartesianNode()), 
 m_gait_planning_test_setup_node(TestSetupGaitPlanningNode()),  
 m_gait_planning_test_joints_node(TestJointsGaitPlanningNode())
{
    
    m_exo_mode_subscriber = create_subscription<march_shared_msgs::msg::ExoMode>("current_mode", 10, std::bind(&GaitPlanningAnglesNode::currentModeCallback, this, _1)); 

    RCLCPP_INFO(this->get_logger(), "Gait Planning Base Node initialized!")

}

void GaitPlanningNode::currentModeCallback(const march_shared_msgs::msg::ExoMode::SharedPtr msg){
    RCLCPP_INFO(this->get_logger(), "Received current mode: %d", msg->mode); 
    // in some way, call the subnodes here. Problem: the subnodes cannot publish anything if they are not spinning as nodes. Do we want to include all publishers here and somehow have the nodes operate separately (without spinning them??) or do we want to include the logic here? 
    switch (msg->gaiting_type){
        case "Cartesian" :
            switch (msg->cameras){
                case true:
                case false: 
            }
        case "Joint Angles" :
            m_gait_planning_joint_angles_node; 
        case "Test Setup" :
            m_gait_planning_test_setup_node; 
        case "Test Joints" :
            m_gait_planning_test_joints_node; 
    }
}
