
#include "rclcpp/rclcpp.hpp" 
#include "march_gait_planning/gait_planning_joint_angles_node.hpp"
#include "march_gait_planning/gait_planning_node.hpp"
#include "march_gait_planning/test_setup_gait_planning_node.hpp"
#include "march_gait_planning/test_joints_gait_planning_node.hpp"
#include "march_shared_msgs/msg/exo_mode.hpp"


class GaitplanningNode:public rclcpp::Node 
{
public: 
    explicit GaitPlanningNode(); 

private: 

    rclcpp::Subscription<march_shared_msgs::msg::ExoMode>::SharedPtr m_exo_mode_subscriber; 

    void currentModeCallback(const march_shared_msgs::msg::ExoMode::SharedPtr msg); 

    GaitPlanningCartesianNode m_gait_planning_cartesian_node; 
    GaitPlanningAnglesNode m_gait_planning_angles_node; 
    TestSetupGaitPlanningNode m_gait_planning_test_joints_node; 
_
};