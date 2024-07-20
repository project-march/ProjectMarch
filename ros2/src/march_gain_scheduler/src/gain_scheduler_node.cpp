#include "gain_scheduler/gain_scheduler_node.hpp"
#include <chrono>

using std::placeholders::_1; 

#define PUBLISH_TIME 10

GainSchedulerNode::GainSchedulerNode() 
    : Node("gain_scheduler_node")
 {
    declare_parameter("system_type", std::string("tsu"));
    std::string system_type = this->get_parameter("system_type").as_string();
                                                    
    m_gain_scheduler = GainScheduler(system_type);

    m_joint_gains_publisher = create_publisher<march_shared_msgs::msg::JointGains>("joint_gains", 10);

    m_mode_subscriber = create_subscription<march_shared_msgs::msg::ExoMode>(
        "current_mode", 10, std::bind(&GainSchedulerNode::currentModeCallback, this, _1));

    m_joint_states_subscriber = create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10, std::bind(&GainSchedulerNode::jointStatesCallback, this, _1));

    m_state_estimation_subscriber = create_subscription<march_shared_msgs::msg::StateEstimation>(
        "state_estimation", 10, std::bind(&GainSchedulerNode::stateEstimationCallback, this, _1));

    m_robot_description_subscriber = create_subscription<std_msgs::msg::String>(
        "robot_description", 10, std::bind(&GainSchedulerNode::robotDescriptionCallback, this, _1));

    m_timer = this->create_wall_timer(std::chrono::milliseconds(PUBLISH_TIME), std::bind(&GainSchedulerNode::publishJointsWithGains, this));
 }

void GainSchedulerNode::currentModeCallback(const march_shared_msgs::msg::ExoMode::SharedPtr msg) {
    m_gain_scheduler.setGaitConfiguration((ExoMode)msg->mode);
}

void GainSchedulerNode::jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    m_latest_joint_state = msg;   
}

void GainSchedulerNode::stateEstimationCallback(const march_shared_msgs::msg::StateEstimation::SharedPtr msg) {
    m_current_stance_leg = msg->current_stance_leg;
}

void GainSchedulerNode::robotDescriptionCallback(const std_msgs::msg::String::SharedPtr msg) {
    m_robot_description = msg;
}

void GainSchedulerNode::publishJointsWithGains() {   
    joints_with_gains joints_with_gains = getJointsWithGains();
    // m_gain_scheduler.calculateInertia(1.0, 1.0);
    // RCLCPP_INFO(this->get_logger(), "Inertia: %f", m_gain_scheduler.calculateInertia(1.0, 1.0));
    // RCLCPP_INFO(this->get_logger(), "Summed inertia: %f", m_gain_scheduler.sumInertia(2.0, 1.0, 3.0, 1.0, 4.0, 1.0));
    // RCLCPP_INFO(this->get_logger(), m_robot_description->data.c_str());
    // printf("Inertia: %f\n", m_gain_scheduler.calculateInertia(1.0, 1.0)); 

    for (size_t i = 0; i < joints_with_gains.size(); ++i) {
        march_shared_msgs::msg::JointGains joint_gains_msg;
        joint_gains_msg.joint_name = std::get<GainScheduler::joint_name_index>(joints_with_gains[i]);
        joint_gains_msg.proportional_gain = std::get<GainScheduler::p_gain_index>(joints_with_gains[i]);
        joint_gains_msg.integral_gain = std::get<GainScheduler::i_gain_index>(joints_with_gains[i]);
        joint_gains_msg.derivative_gain = std::get<GainScheduler::d_gain_index>(joints_with_gains[i]);
        m_joint_gains_publisher->publish(joint_gains_msg);
    }
}   

joints_with_gains GainSchedulerNode::getJointsWithGains() {
    joints_with_gains joints_with_gains;

    if (m_gain_scheduler.m_scheduling_variable == "constant_gains") {
        joints_with_gains = m_gain_scheduler.getConstantGains("constant_gains");
    } else if (m_gain_scheduler.m_scheduling_variable == "joint_angle_gains") {
        joints_with_gains = m_gain_scheduler.getJointAngleGains(m_latest_joint_state);
    }

    if (m_gain_scheduler.m_use_stance_swing_leg_gains) {
        joints_with_gains = m_gain_scheduler.setStanceSwingLegGains(joints_with_gains, m_current_stance_leg);
    }
    return joints_with_gains;
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GainSchedulerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}



