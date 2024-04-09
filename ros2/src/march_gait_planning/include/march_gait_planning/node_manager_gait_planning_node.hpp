#include "rclcpp/rclcpp.hpp" 
#include "march_shared_msgs/msg/exo_mode.hpp"
#include "march_gait_planning/gait_planning_joint_angles_node.hpp"

class NodeManagerGaitPlanning : public rclcpp::Node {
    public: 
    
    explicit NodeManagerGaitPlanning(); 

    private: 

    void activationCallback(const march_shared_msgs::msg::ExoMode::SharedPtr msg); 
    void activateGaitPlanningAnglesNode(); 
    // void activateGaitPlanningCartesianNode(); 
    // void activateGaitPlanningTestJointsNode(); 
    // void activateGaitPlanningTestSetupNode(); 

    rclcpp::Subscription<march_shared_msgs::msg::ExoMode>::SharedPtr m_mode_subscriber; 

}; 