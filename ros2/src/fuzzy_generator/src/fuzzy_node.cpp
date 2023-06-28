//
// Created by rixt on 12-4-23.
//

#include "fuzzy_generator/fuzzy_node.hpp"
#include "fuzzy_generator/fuzzy_generator.hpp"
using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

FuzzyNode::FuzzyNode()
        : Node("fuzzy_node")
        , m_fuzzy_generator()
{
    // m_stance_leg_subscription = this->create_subscription<std_msgs::msg::Int32>(
    //         "current_stance_foot", 10, std::bind(&FuzzyNode::stance_leg_callback, this, _1));
    m_foot_height_subscription = this->create_subscription<march_shared_msgs::msg::FeetHeightStamped>(
            "robot_feet_height", 10, std::bind(&FuzzyNode::height_callback, this, _1));

    m_control_type_subscription = this->create_subscription<std_msgs::msg::String>(
            "/march/weight_control_type", 10, std::bind(&FuzzyNode::control_type_callback, this, _1));

    m_weight_publisher = this->create_publisher<march_shared_msgs::msg::WeightStamped>("fuzzy_weight", 10);

    this->declare_parameter("allowed_control_type", "fuzzy");
}

/**
 * Sets the height of both feet
 *
 * @param msg Message that contains the height of both feet. Updates and publishes new weights accordingly.
 * @return
 */
void FuzzyNode::height_callback(march_shared_msgs::msg::FeetHeightStamped::SharedPtr msg){

    float left_foot_height = msg->heights[0];
    float right_foot_height = msg->heights[1];

    if(left_foot_height >= m_fuzzy_generator.getLowerBound() && left_foot_height <= m_fuzzy_generator.getUpperBound()){
        auto weights = m_fuzzy_generator.calculateWeights("left", left_foot_height);
        for(auto w: weights){

            // send the weights for the left leg
            march_shared_msgs::msg::WeightStamped left_weights;
            left_weights.joint_name = std::get<0>(w);
            left_weights.position_weight = std::get<1>(w);
            left_weights.torque_weight = std::get<2>(w);
            left_weights.header.frame_id = this->get_name();
            publish_weights(left_weights);
        }
    }

    if(right_foot_height >= m_fuzzy_generator.getLowerBound() && right_foot_height <= m_fuzzy_generator.getUpperBound()){
        auto weights = m_fuzzy_generator.calculateWeights("right", right_foot_height);
        for(auto w: weights){

            // send the weights for the right leg
            march_shared_msgs::msg::WeightStamped right_weights;
            right_weights.joint_name = std::get<0>(w);
            right_weights.position_weight = std::get<1>(w);
            right_weights.torque_weight = std::get<2>(w);
            right_weights.header.frame_id = this->get_name();
            publish_weights(right_weights);
        }
    }
}

/**
 * Sets the allowed type of weights for torque and position
 *
 * @param msg Message that contains the control type: Position, Torque, or Fuzzy
 * @return
 */
void FuzzyNode::control_type_callback(std_msgs::msg::String::SharedPtr msg) {
    std::string allowed_control_type = msg->data;

    std::string prev_allowed_control_type = this->get_parameter("allowed_control_type").as_string();
    if((prev_allowed_control_type == "position" && allowed_control_type == "torque") || (prev_allowed_control_type == "torque" && allowed_control_type == "position")){
        RCLCPP_FATAL_STREAM(this->get_logger(), "switching from position to torque causes shooting!");
        throw(std::invalid_argument("SHOULD NOT YET SWITCH BETWEEN POSITION AND CONTROL!"));
    }

    if(allowed_control_type != "fuzzy" && allowed_control_type != "torque" && allowed_control_type != "position"){
        RCLCPP_WARN_STREAM(this->get_logger(), "NOT A RECOGNIZED CONTROL TYPE: " << allowed_control_type);
    }
    else{
        RCLCPP_INFO_STREAM(this->get_logger(), "setting control type to " << allowed_control_type << " control ");
    }

    this->set_parameter(rclcpp::Parameter("allowed_control_type", allowed_control_type));

    // dummy message to make sure that the fuzzy node publishes weights after setting a control type
    march_shared_msgs::msg::WeightStamped weights;
    weights.torque_weight = 0;
    weights.position_weight = 0;
    publish_weights(weights);
}

void FuzzyNode::publish_weights(march_shared_msgs::msg::WeightStamped msg){

    std::string allowed_control_type = this->get_parameter("allowed_control_type").as_string();
    RCLCPP_INFO_STREAM(this->get_logger(), "setting weights according to " << allowed_control_type << " control ");

    if(allowed_control_type == "position"){
        msg.position_weight = 1;
        msg.torque_weight = 0;
        m_weight_publisher->publish(msg);
    }
    else if(allowed_control_type == "torque"){
        msg.position_weight = 0;
        msg.torque_weight = 1;
        m_weight_publisher->publish(msg);
    }
    else if(allowed_control_type == "fuzzy"){
        m_weight_publisher->publish(msg);
    }
    else{
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