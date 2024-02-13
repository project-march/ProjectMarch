//
// Created by rixt on 12-4-23.
//

#include "fuzzy_generator/fuzzy_generator_node.hpp"

using std::placeholders::_1;

FuzzyGeneratorNode::FuzzyGeneratorNode()
    : Node("fuzzy_generator_node")
{
    declare_parameter("config_path", std::string("src/march_fuzzy_generator/config/default_weights.yaml"));
    std::string config_path = this->get_parameter("config_path").as_string();
    m_fuzzy_generator = FuzzyGenerator(config_path);

    // TODO: let the state estimator publish the foot heights
    m_foot_height_subscription = this->create_subscription<march_shared_msgs::msg::FootHeights>(
        "robot_foot_heights", 10, std::bind(&FuzzyGeneratorNode::footHeightsCallback, this, _1));

    // TODO: let something (?) publish the control type
    m_control_type_subscription = this->create_subscription<std_msgs::msg::String>(
        "low_level_control_type", 10, std::bind(&FuzzyGeneratorNode::controlTypeCallback, this, _1));

    m_mode_subscription = create_subscription<march_shared_msgs::msg::ExoMode>(
        "current_mode", 10, std::bind(&FuzzyGeneratorNode::currentModeCallback, this, _1));

    m_weight_publisher = this->create_publisher<march_shared_msgs::msg::FuzzyWeights>("fuzzy_weights", 10);

    m_timer = create_wall_timer(std::chrono::milliseconds(1000), std::bind(&FuzzyGeneratorNode::publishFuzzyWeights, this));
}


// Method to receive the foot heights
void FuzzyGeneratorNode::footHeightsCallback(const march_shared_msgs::msg::FootHeights::SharedPtr msg){
    if (msg != nullptr) {
        m_latest_foot_heights = msg;
    } else {
        RCLCPP_ERROR(get_logger(), "Received nullptr foot heights");
    }
}


// Method to receive the control type, etiher "position" or "fuzzy"
void FuzzyGeneratorNode::controlTypeCallback(std_msgs::msg::String::SharedPtr msg){
    if (msg != nullptr) {
        m_control_type = msg->data;
    } else {
        m_control_type = "position";
        RCLCPP_WARN(get_logger(), "Received nullptr control type, defaulting to position control");
    }
}


// Method to receive the current mode 
void FuzzyGeneratorNode::currentModeCallback(const march_shared_msgs::msg::ExoMode::SharedPtr msg) {
    RCLCPP_INFO(get_logger(), "Received current mode: %d", msg->mode);
    m_fuzzy_generator.setConfigPath((exoMode)msg->mode);
}


// Method to publish the fuzzy weights
void FuzzyGeneratorNode::publishFuzzyWeights(){

    march_shared_msgs::msg::FuzzyWeights fuzzy_weights_msg;

    if (m_control_type == "yomama") {
        for (const auto& joint_names : m_fuzzy_generator.m_joint_names){
            fuzzy_weights_msg.joint_name = joint_names; 
            fuzzy_weights_msg.position_weight = 1.0f;
            fuzzy_weights_msg.torque_weight = 0.0f;
            m_weight_publisher->publish(fuzzy_weights_msg);
        }
    } else {
        // Uncomment desired method
        const auto fuzzy_weights = m_fuzzy_generator.getConstantWeights();
        // const auto fuzzy_weights = m_fuzzy_generator.calculateFootHeightWeights(m_latest_foot_heights);
        // const auto fuzzy_weights = m_fuzzy_generator.calculateStanceSwingLegWeights(0, 0);

        march_shared_msgs::msg::FuzzyWeights fuzzy_weights_msg;
        for (const auto& weight : fuzzy_weights) {
            fuzzy_weights_msg.joint_name = std::get<m_joint_name_index>(weight);
            fuzzy_weights_msg.position_weight = std::get<m_position_weight_index>(weight);
            fuzzy_weights_msg.torque_weight = std::get<m_torque_weight_index>(weight);
            m_weight_publisher->publish(fuzzy_weights_msg);
        }
    }
}


int main(int argc, char** argv){

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FuzzyGeneratorNode>());
    rclcpp::shutdown();
    return 0;
}