#include <march_gazebo_plugins/com_controller_node.h>

ComControllerNode::ComControllerNode(
    std::shared_ptr<gazebo::ObstacleController> controller, std::map<std::string, int>& pd_values)
    : Node("com_controller_node", "march")
    , controller_(std::move(controller))
    , pd_values_(pd_values)
    , balance_(false)
{
    // NOLINTBEGIN
    // bugprone-argument-comment: message_queue_length is more accurate then qos.
    // performance-unnecessary-value-param: This is needed the way ros is set up.
    subscription_ = this->create_subscription<march_shared_msgs::msg::CurrentGait>(
        /*topic_name=*/"/march/gait_selection/current_gait", /*message_queue_length=*/1,
        /*callback=*/[this](const march_shared_msgs::msg::CurrentGait::SharedPtr msg) {
            controller_->newSubgait(msg);
        });
    // NOLINTEND

    for (auto const& [key, val] : pd_values_) {
        this->declare_parameter<int>(key, val);
    }
    this->declare_parameter<bool>("balance", balance_);

    callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&ComControllerNode::parameter_callback, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult ComControllerNode::parameter_callback(
    const std::vector<rclcpp::Parameter>& parameters)
{

    for (const rclcpp::Parameter& parameter : parameters) {
        if (parameter.get_type() == rclcpp::PARAMETER_INTEGER) {
            pd_values_[parameter.get_name()] = parameter.as_int();
            RCLCPP_WARN(this->get_logger(), "%s was set to %i", parameter.get_name().c_str(), parameter.as_int());
        }
        if (parameter.get_type() == rclcpp::PARAMETER_BOOL) {
            balance_ = parameter.as_bool();
            RCLCPP_WARN(this->get_logger(), "%s was set to %i", parameter.get_name().c_str(), parameter.as_bool());
        }
    }

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    return result;
}

bool ComControllerNode::balance_mode()
{
    return balance_;
}