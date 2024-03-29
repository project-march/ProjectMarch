//
// Created by rixt on 12-4-23.
//

#include "fuzzy_generator/fuzzy_node.hpp"
#include "fuzzy_generator/fuzzy_generator.hpp"
#include <chrono>
using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

FuzzyNode::FuzzyNode()
    : Node("fuzzy_node")
{
    declare_parameter("config_path", std::string(""));
    std::string config_path = this->get_parameter("config_path").as_string();
    m_fuzzy_generator = FuzzyGenerator(config_path);

    m_foot_height_subscription = this->create_subscription<march_shared_msgs::msg::FeetHeightStamped>(
        "robot_feet_height", 10, std::bind(&FuzzyNode::height_callback, this, _1));

    m_control_type_subscription = this->create_subscription<std_msgs::msg::String>(
        "/march/weight_control_type", 10, std::bind(&FuzzyNode::control_type_callback, this, _1));

    m_weight_publisher = this->create_publisher<march_shared_msgs::msg::WeightStamped>("fuzzy_weight", 10);

    this->declare_parameter("allowed_control_type", "position");
}

/**
 * Sets the height of both feet
 *
 * @param msg Message that contains the height of both feet. Updates and publishes new weights accordingly.
 * @return
 */
void FuzzyNode::height_callback(march_shared_msgs::msg::FeetHeightStamped::SharedPtr msg)
{

    auto weights = m_fuzzy_generator.calculateWeights(msg->heights);
    for (auto w : weights) {

        // send the weights for the legs
        march_shared_msgs::msg::WeightStamped fuzzy_weights;
        fuzzy_weights.joint_name = std::get<0>(w);
        fuzzy_weights.position_weight = std::get<1>(w);
        fuzzy_weights.torque_weight = std::get<2>(w);
        fuzzy_weights.header.frame_id = this->get_name();
        publish_weights(fuzzy_weights);
    }
    // RCLCPP_INFO_STREAM(this->get_logger(), "--------------------------------------");
}

/**
 * Sets the allowed type of weights for torque and position
 *
 * @param msg Message that contains the control type: Position, Torque, or Fuzzy
 * @return
 */
void FuzzyNode::control_type_callback(std_msgs::msg::String::SharedPtr msg)
{
    std::string allowed_control_type = msg->data;

    if (allowed_control_type != "fuzzy" && allowed_control_type != "position") {
        RCLCPP_WARN_STREAM(this->get_logger(), "NOT A RECOGNIZED CONTROL TYPE: " << allowed_control_type);
        return;
    } else {
        RCLCPP_INFO_STREAM(this->get_logger(), "setting control type to " << allowed_control_type << " control ");
    }

    this->set_parameter(rclcpp::Parameter("allowed_control_type", allowed_control_type));

    // RCLCPP_INFO_STREAM(this->get_logger(), "--------------------------------------");
}

void FuzzyNode::publish_weights(march_shared_msgs::msg::WeightStamped msg)
{

    std::string allowed_control_type = this->get_parameter("allowed_control_type").as_string();

    if (allowed_control_type.compare("position") == 0) {
        msg.position_weight = 1.0f;
        msg.torque_weight = 0.0f;
        m_weight_publisher->publish(msg);
        // RCLCPP_INFO_STREAM(this->get_logger(), "published: " << msg.joint_name << " " << msg.position_weight << " "
        // << msg.torque_weight); RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "published: %s
        // position: %f torque: %f", msg.joint_name.c_str(), msg.position_weight, msg.torque_weight);
    } else if (allowed_control_type.compare("fuzzy") == 0) {
        m_weight_publisher->publish(msg);
        // RCLCPP_INFO_STREAM(this->get_logger(), "published: " << msg.joint_name << " " << msg.position_weight << " "
        // << msg.torque_weight); RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "published: %s
        // position: %f torque: %f", msg.joint_name.c_str(), msg.position_weight, msg.torque_weight);
    } else {
        RCLCPP_WARN_STREAM(this->get_logger(), "NOT A RECOGNIZED CONTROL TYPE: " << allowed_control_type);
    }
}

/**
 * Main function to run the node.
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FuzzyNode>());

    rclcpp::shutdown();
    return 0;
}