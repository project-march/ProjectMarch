#include "rclcpp/rclcpp.hpp" 
#include "march_shared_msgs/msg/exo_mode.hpp"
#include "march_gait_planning/gait_planning_joint_angles_node.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"

class NodeManagerGaitPlanning : public rclcpp::Node {
    public: 
    
    explicit NodeManagerGaitPlanning(); 

    //FILPMJE
    void messageCallback(const std_msgs::msg::String::SharedPtr msg); 
    void notificationCallback(const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg); 
    // 

    private: 

    //FILMPJE 
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::String>> m_message_subscriber; 
    std::shared_ptr<rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>> m_notification_subscriber; 
    //

    void activationCallback(const march_shared_msgs::msg::ExoMode::SharedPtr msg); 
    void activateAnglesNode(const march_shared_msgs::msg::ExoMode::SharedPtr msg); 
    void deactivateAnglesNode(); 
 
    rclcpp::Subscription<march_shared_msgs::msg::ExoMode>::SharedPtr m_mode_subscriber; 

    std::shared_ptr<GaitPlanningAnglesNode> m_angles_node; 
    
}; 