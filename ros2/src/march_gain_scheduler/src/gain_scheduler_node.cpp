#include "gain_scheduler/gain_scheduler_node.hpp"
#include <chrono>

using std::placeholders::_1; 

GainSchedulerNode::GainSchedulerNode()
 : Node("gain_scheduler_node")
 {
    this->declare_parameter<std::string>("config_path", "src/march_gain_scheduler/config/walk_gains.yaml");   
    auto config_path = this->get_parameter("config_path").as_string();                                                       
    m_scheduler = GainScheduler(config_path);

    m_pid_values_publisher = create_publisher<march_shared_msgs::msg::PidValues>("pid_values", 10);

    m_mode_subscriber = create_subscription<march_shared_msgs::msg::ExoMode>(
        "current_mode", 10, std::bind(&GainSchedulerNode::currentModeCallback, this, _1));


    m_joint_states_subscriber = create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10, std::bind(&GainSchedulerNode::jointStatesCallback, this, _1));

    m_timer = create_wall_timer(std::chrono::milliseconds(1000), std::bind(&GainSchedulerNode::timerCallback, this));
 }

void GainSchedulerNode::setTimer(int publish_time) {
    m_timer->cancel();
    m_timer = create_wall_timer(std::chrono::milliseconds(publish_time), std::bind(&GainSchedulerNode::timerCallback, this));
}


void GainSchedulerNode::currentModeCallback(const march_shared_msgs::msg::ExoMode::SharedPtr msg) {
    RCLCPP_INFO(get_logger(), "Received current mode: %s", toString(static_cast<exoMode>(msg->mode)).c_str());
    m_scheduler.setConfigPath((exoMode)msg->mode);
}

std::string GainSchedulerNode::vectorToString(const std::vector<double>& vec) {
    std::ostringstream oss;
    std::copy(vec.begin(), vec.end(), std::ostream_iterator<double>(oss, " "));
    return oss.str();
}

void GainSchedulerNode::jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (msg != nullptr) {
        // Convert the message to a string and print it
        std::string position = vectorToString(msg->position);
        std::string velocity = vectorToString(msg->velocity);
        RCLCPP_INFO(get_logger(), "Position: %s, Velocity: %s", position.c_str(), velocity.c_str());
        
        m_latest_joint_state = msg;   
    } 
}

void GainSchedulerNode::publishPidValues() {   
    std::vector<std::tuple<std::string, double, double, double>> joints;
    const unsigned int joint_name = 0;
    const unsigned int joint_p_gain = 1;
    const unsigned int joint_i_gain = 2;
    const unsigned int joint_d_gain = 3;

    if (m_scheduler.isInterpolating()) {
        joints = m_scheduler.getInterpolatedPidValues();
        RCLCPP_INFO(get_logger(), "Interpolating PID values");
    } else {
        if (m_latest_joint_state != nullptr) {
            joints = m_scheduler.getAllJointStatePidValues(m_latest_joint_state);
            RCLCPP_INFO(get_logger(), "Using joint states from message");
        } else {
            joints = m_scheduler.getAllPidValues();
            RCLCPP_INFO(get_logger(), "Using default joint states");
        }
    }

    for (const auto& joint : joints) {
        march_shared_msgs::msg::PidValues pid_values_msg;
        pid_values_msg.joint_name = std::get<joint_name>(joint);
        pid_values_msg.proportional_gain = std::get<joint_p_gain>(joint);
        pid_values_msg.integral_gain = std::get<joint_i_gain>(joint);
        pid_values_msg.derivative_gain = std::get<joint_d_gain>(joint);

        m_pid_values_publisher->publish(pid_values_msg);

        RCLCPP_INFO(get_logger(), "Published PID values for joint: %s, P: %f, I: %f, D: %f",
                    pid_values_msg.joint_name.c_str(), pid_values_msg.proportional_gain,
                    pid_values_msg.integral_gain, pid_values_msg.derivative_gain);
    }
        RCLCPP_INFO(get_logger(), "                                    ");
}   


void GainSchedulerNode::timerCallback() {
    
    if (m_scheduler.isInterpolating()) {
        setTimer(m_scheduler.m_publish_time);
        
        if (m_scheduler.m_time_step < m_scheduler.m_total_time) {
            m_scheduler.incrementTimeStep();
            RCLCPP_INFO(get_logger(), "Time step: %f", m_scheduler.m_time_step);
        } else {
            m_scheduler.stopInterpolation();
            m_scheduler.m_time_step = 0;
        }
    } else {
        setTimer(1000);  
    }
    
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



