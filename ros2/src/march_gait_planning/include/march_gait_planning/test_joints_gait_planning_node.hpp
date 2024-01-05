#ifndef TEST_GAIT_PLANNING_NODE_HPP
#define TEST_GAIT_PLANNING_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "march_gait_planning/test_setup_gait_planning.hpp"
#include "march_shared_msgs/msg/exo_state.hpp"
#include "march_shared_msgs/msg/exo_state_and_joint.hpp"

class TestJointsGaitPlanningNode : public rclcpp::Node
{
public:
    explicit TestJointsGaitPlanningNode();
    ~TestJointsGaitPlanningNode() = default;


private:
    void currentStateCallback(const march_shared_msgs::msg::ExoStateAndJoint::SharedPtr msg);
    void footPositionsPublish();
    void timerCallback();
    

    void setActuatedJoint(const std::string &actuated_joint);
    std::string getActuatedJoint() const;

    rclcpp::Subscription<march_shared_msgs::msg::ExoStateAndJoint>::SharedPtr m_exo_state_subscriber;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr m_test_joint_trajectory_controller_state_pub_;
    rclcpp::TimerBase::SharedPtr m_timer;

    TestSetupGaitPlanning m_gait_planning;
    std::vector<double> m_current_trajectory;
    trajectory_msgs::msg::JointTrajectory::SharedPtr m_current_joint_angles_msg;
    std::string m_actuated_joint;
};

#endif // TEST_GAIT_PLANNING_NODE_HPP
