#include <march_gazebo_plugins/com_controller_node.h>

ComControllerNode::ComControllerNode(
    std::shared_ptr<gazebo::ObstacleController> controller,
    std::map<std::string, int>& pd_values)
    : Node("com_controller_node", "march")
    , controller_(controller)
    , pd_values_(pd_values)
    , balance_(false)
{
    subscription_
        = this->create_subscription<march_shared_msgs::msg::CurrentGait>(
            "/march/gait_selection/current_gait", 1,
            std::bind(&ComControllerNode::topic_callback, this,
                std::placeholders::_1));

    for (auto const& [key, val] : pd_values_) {
        this->declare_parameter<int>(key, val);
    }
    this->declare_parameter<bool>("balance", balance_);

    callback_handle_ = this->add_on_set_parameters_callback(std::bind(
        &ComControllerNode::parameter_callback, this, std::placeholders::_1));
}

void ComControllerNode::topic_callback(
    const march_shared_msgs::msg::CurrentGait::SharedPtr msg) const
{
    controller_->newSubgait(msg);
}

rcl_interfaces::msg::SetParametersResult ComControllerNode::parameter_callback(
    const std::vector<rclcpp::Parameter>& parameters)
{

    for (rclcpp::Parameter parameter : parameters) {
        if (parameter.get_type() == rclcpp::PARAMETER_INTEGER) {
            pd_values_[parameter.get_name()] = parameter.as_int();
            RCLCPP_WARN(this->get_logger(), "%s was set to %i",
                parameter.get_name().c_str(), parameter.as_int());
        }
        if (parameter.get_type() == rclcpp::PARAMETER_BOOL) {
            balance_ = parameter.as_bool();
            RCLCPP_WARN(this->get_logger(), "%s was set to %i",
                parameter.get_name().c_str(), parameter.as_bool());
        }
    }

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    return result;
}

bool ComControllerNode::balance_mode(){
    return balance_;
}