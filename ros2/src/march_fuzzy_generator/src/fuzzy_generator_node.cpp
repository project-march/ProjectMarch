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

    m_torque_subscription = this->create_subscription<march_shared_msgs::msg::JointMotorControllerState>(
        "joint_motor_controller_state", 10, std::bind(&FuzzyGeneratorNode::measuredTorqueCallback, this, _1));

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
    m_fuzzy_generator.setConfigPath((ExoMode)msg->mode);
}

// Method to receive the measured torques from the motor controller state broadcaster
void FuzzyGeneratorNode::measuredTorqueCallback(const march_shared_msgs::msg::JointMotorControllerState::SharedPtr msg){
    m_left_ankle_torque = getSpecificJointTorque(msg, "left_ankle");
    m_right_ankle_torque = getSpecificJointTorque(msg, "right_ankle");
}

double FuzzyGeneratorNode::getSpecificJointTorque(const march_shared_msgs::msg::JointMotorControllerState::SharedPtr msg, const std::string& specific_joint) {
    if (msg->joint_name == specific_joint) {
        return msg->torque;
    } else {
        return 0;
    }
}

// Method to publish the fuzzy weights
void FuzzyGeneratorNode::publishFuzzyWeights(){

    std_msgs::msg::Float64MultiArray fuzzy_weights_msg;
    std::vector<std::tuple<std::string, float, float>> fuzzy_weights;

    if (m_fuzzy_generator.m_control_type == "position") {
        fuzzy_weights = m_fuzzy_generator.returnPositionWeights();
    } else if (m_fuzzy_generator.m_control_type == "constant"){
        fuzzy_weights = m_fuzzy_generator.getConstantWeights();
    } else if (m_fuzzy_generator.m_control_type == "foot_heights") {
        fuzzy_weights = m_fuzzy_generator.calculateFootHeightWeights(m_latest_foot_heights);
    } else if (m_fuzzy_generator.m_control_type == "ankle_torques") {
        fuzzy_weights = m_fuzzy_generator.getAnkleTorques(m_left_ankle_torque, m_right_ankle_torque);
    }
    
    for (const auto& weight : fuzzy_weights) {
        double torque_weight = std::get<m_torque_weight_index>(weight);
        fuzzy_weights_msg.data.push_back(torque_weight);
    }
    m_weight_publisher->publish(fuzzy_weights_msg);
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FuzzyGeneratorNode>());
    rclcpp::shutdown();
    return 0;
}