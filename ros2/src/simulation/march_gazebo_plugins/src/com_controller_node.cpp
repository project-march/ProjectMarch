#include <march_gazebo_plugins/com_controller_node.h>

ComControllerNode::ComControllerNode(std::shared_ptr<gazebo::ObstacleController> controller)
    : Node("com_controller_node", "march"),
    controller_(controller)
{
    subscription_
        = this->create_subscription<march_shared_msgs::msg::CurrentGait>(
            "/march/gait_selection/current_gait", 1,
            std::bind(&ComControllerNode::topic_callback, this,
                std::placeholders::_1));
}

void ComControllerNode::topic_callback(
    const march_shared_msgs::msg::CurrentGait::SharedPtr msg) const
{
    controller_->newSubgait(msg);
}