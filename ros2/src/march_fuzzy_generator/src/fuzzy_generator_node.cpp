#include "fuzzy_generator/fuzzy_generator_node.hpp"

using std::placeholders::_1;

FuzzyGeneratorNode::FuzzyGeneratorNode()
    : Node("fuzzy_generator_node")
{
    declare_parameter("config_path", std::string("~/src/march_fuzzy_generator/config/default_weights.yaml"));
    std::string config_path = this->get_parameter("config_path").as_string();
    m_fuzzy_generator = FuzzyGenerator(config_path);

    // TODO: let the state estimator publish the foot heights
    m_foot_height_subscription = this->create_subscription<march_shared_msgs::msg::FeetHeightStamped>(
        "robot_foot_heights", 10, std::bind(&FuzzyGeneratorNode::footHeightsCallback, this, _1));

    m_torque_subscription = this->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
        "measured_torque", 10, std::bind(&FuzzyGeneratorNode::measuredTorquesCallback, this, _1));

    m_mode_subscription = this->create_subscription<march_shared_msgs::msg::ExoMode>(
        "current_mode", 10, std::bind(&FuzzyGeneratorNode::currentModeCallback, this, _1));

    m_weight_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>("march_fuzzy_weights_controller/commands", rclcpp::SystemDefaultsQoS());

    m_timer = create_wall_timer(std::chrono::milliseconds(2000), std::bind(&FuzzyGeneratorNode::publishFuzzyWeights, this));
}


// Method to receive the foot heights
void FuzzyGeneratorNode::footHeightsCallback(const march_shared_msgs::msg::FeetHeightStamped::SharedPtr msg){
    if (msg != nullptr) {
        m_latest_foot_heights = msg;
    } else {
        RCLCPP_ERROR(get_logger(), "Received nullptr foot heights");
    }
}


// Method to receive the current mode 
void FuzzyGeneratorNode::currentModeCallback(const march_shared_msgs::msg::ExoMode::SharedPtr msg) {
    // RCLCPP_INFO(get_logger(), "Received current mode: %d", msg->mode);
    m_fuzzy_generator.setConfigPath((ExoMode)msg->mode);
}


// Method to receive the measured torques from the hardware interface
void FuzzyGeneratorNode::measuredTorquesCallback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg){

    m_left_ankle_torque = getActualJointTorque(msg, "left_ankle");
    m_right_ankle_torque = getActualJointTorque(msg, "right_ankle");
}


// TODO: get torque values from the motor controller state broadcaster (has to be tested itself first)
double FuzzyGeneratorNode::getActualJointTorque(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr& msg, const std::string& joint_name) {
    for (size_t i = 0; i < msg->joint_names.size(); ++i) {
        if (msg->joint_names[i] == joint_name) {
            RCLCPP_INFO(get_logger(), "The actual torque of the %s joint is: %f", joint_name.c_str(), msg->actual.effort[i]);
            return msg->actual.effort[i];
        }
    }
    RCLCPP_ERROR(get_logger(), "The joint %s is not found in the message", joint_name.c_str());
    return 0.0f;
}


// Method to publish the fuzzy weights
void FuzzyGeneratorNode::publishFuzzyWeights(){

    std_msgs::msg::Float64MultiArray fuzzy_weights_msg;
    std::vector<std::tuple<std::string, float, float>> fuzzy_weights;

    if (m_fuzzy_generator.m_control_type == "position") {
        size_t num_joints = m_fuzzy_generator.m_joint_names.size();
        fuzzy_weights_msg.data.push_back(0.01);
    } else if (m_fuzzy_generator.m_control_type == "constant"){
        fuzzy_weights = m_fuzzy_generator.getConstantWeights();
    } else if (m_fuzzy_generator.m_control_type == "foot_height") {
        fuzzy_weights = m_fuzzy_generator.calculateFootHeightWeights(m_latest_foot_heights);
    } else if (m_fuzzy_generator.m_control_type == "stance_swing_leg") {
        fuzzy_weights = m_fuzzy_generator.calculateStanceSwingLegWeights(m_left_ankle_torque, m_right_ankle_torque);
    }
    
    for (const auto& weight : fuzzy_weights) {
        double torque_weight = std::get<m_torque_weight_index>(weight);
        fuzzy_weights_msg.data.push_back(torque_weight);
    }
    m_weight_publisher->publish(fuzzy_weights_msg);
}



int main(int argc, char** argv){

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FuzzyGeneratorNode>());
    rclcpp::shutdown();
    return 0;
}