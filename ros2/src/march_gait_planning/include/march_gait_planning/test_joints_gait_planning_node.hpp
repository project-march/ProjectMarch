#ifndef TEST_GAIT_PLANNING_NODE_HPP
#define TEST_GAIT_PLANNING_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "march_gait_planning/test_setup_gait_planning.hpp"
#include "march_shared_msgs/msg/exo_mode.hpp"
#include "march_shared_msgs/msg/exo_mode_and_joint.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "march_shared_msgs/msg/state_estimation.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "march_shared_msgs/msg/state_estimation.hpp"

class TestJointsGaitPlanningNode : public rclcpp::Node
{
public:
    explicit TestJointsGaitPlanningNode();
    ~TestJointsGaitPlanningNode() = default;


private:
    void currentModeCallback(const march_shared_msgs::msg::ExoModeAndJoint::SharedPtr msg);
    void footPositionsPublish();
    void timerCallback();
    

    void setActuatedJoint(const std::string &actuated_joint);
    int getActuatedJoint() const;

    void currentJointAnglesCallback(const march_shared_msgs::msg::StateEstimation::SharedPtr msg);
    void processHomeStandGait();

    rclcpp::Subscription<march_shared_msgs::msg::StateEstimation>::SharedPtr m_current_state_subscriber; 
    rclcpp::Subscription<march_shared_msgs::msg::ExoModeAndJoint>::SharedPtr m_exo_mode_subscriber;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_joint_angle_trajectory_publisher; 

    TestSetupGaitPlanning m_gait_planning;
    std::vector<double> m_current_trajectory;
    std_msgs::msg::Float64MultiArray m_joints_msg;
    int m_actuated_joint;
    bool m_first_stand;
    int m_counter;
    std::vector<int> m_mapping;

    std::vector<double> m_home_stand;
};

#endif // TEST_GAIT_PLANNING_NODE_HPP
