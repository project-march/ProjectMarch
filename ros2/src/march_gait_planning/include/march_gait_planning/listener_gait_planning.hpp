#include "rclcpp/rclcpp.hpp" 
#include "march_shared_msgs/msg/exo_mode.hpp"
#include "march_gait_planning/gait_planning_joint_angles_node.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

class NodeManagerGaitPlanning : public rclcpp::Node {
    public: 
    
    explicit NodeManagerGaitPlanning(); 

    void anglesNotificationCallback(const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg); 
    void cartesianNotificationCallback(const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg); 
    // void anglesStateCallback(const lifecycle_msgs::srv::GetState::SharedPtr msg); 
    // void cartesianStateCallback(const lifecycle_msgs::srv::GetState::SharedPtr msg); 

    private: 

    // std::shared_ptr<rclcpp::Subscription<lifecycle_msgs::srv::GetState>> m_angles_state_subscriber; 
    // std::shared_ptr<rclcpp::Subscription<lifecycle_msgs::srv::GetState>> m_cartesian_state_subscriber; 
    std::shared_ptr<rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>> m_angles_notification_subscriber; 
    std::shared_ptr<rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>> m_cartesian_notification_subscriber; 

    rclcpp::Subscription<march_shared_msgs::msg::ExoMode>::SharedPtr m_mode_subscriber; 

    std::shared_ptr<GaitPlanningAnglesNode> m_angles_node; 
    
}; 