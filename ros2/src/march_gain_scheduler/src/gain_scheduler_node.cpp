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

    m_pid_values_publisher = create_publisher<march_shared_msgs::msg::PidValues>("pid_values", 10);

    m_mode_subscriber = create_subscription<march_shared_msgs::msg::ExoMode>(
        "current_mode", 10, std::bind(&GainSchedulerNode::currentModeCallback, this, _1));

    m_joint_states_subscriber = create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10, std::bind(&GainSchedulerNode::jointStatesCallback, this, _1));

    m_state_estimation_subscriber = create_subscription<march_shared_msgs::msg::StateEstimation>(
        "state_estimation", 10, std::bind(&GainSchedulerNode::stateEstimationCallback, this, _1));

    m_timer = create_wall_timer(std::chrono::milliseconds(PUBLISH_TIME), std::bind(&GainSchedulerNode::timerCallback, this));
 }

void GainSchedulerNode::currentModeCallback(const march_shared_msgs::msg::ExoMode::SharedPtr msg) {
    m_gain_scheduler.setConfigPath((ExoMode)msg->mode);
}

void GainSchedulerNode::jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    m_latest_joint_state = msg;   
}

void GainSchedulerNode::stateEstimationCallback(const march_shared_msgs::msg::StateEstimation::SharedPtr msg) {
    m_gain_scheduler.m_current_stance_leg = msg->current_stance_leg;
}

void GainSchedulerNode::publishPidValues() {   
    std::vector<std::tuple<std::string, double, double, double>> joints;
    const unsigned int joint_name = 0;
    const unsigned int joint_p_gain = 1;
    const unsigned int joint_i_gain = 2;
    const unsigned int joint_d_gain = 3;


    for (const auto& joint : joints) {
        march_shared_msgs::msg::PidValues pid_values_msg;
        pid_values_msg.joint_name = std::get<joint_name>(joint);
        pid_values_msg.proportional_gain = std::get<joint_p_gain>(joint);
        pid_values_msg.integral_gain = std::get<joint_i_gain>(joint);
        pid_values_msg.derivative_gain = std::get<joint_d_gain>(joint);

        m_pid_values_publisher->publish(pid_values_msg);
    }
}   


void GainSchedulerNode::timerCallback() {
        
    publishPidValues();
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GainSchedulerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}



