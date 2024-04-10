/*Authors: Andrew Hutani, MIX

This node is a seperate mode Machine for the testing of single joints. It only uses modes BootUp, Stand, and Walk, where BootUp is used to switch joints and 
Walk to send a sinusoidal wave to the joint.
This node will send the actuated joint alongside the mode to the gait planning node.

This node is only called in the test_joints launch file.

*/

#include "march_mode_machine/test_joints_mode_machine_node.hpp"
using std::placeholders::_1;
using std::placeholders::_2;


TestJointsModeMachineNode::TestJointsModeMachineNode()
:   Node("mode_machine_node"),
    m_mode_machine(TestJointsModeMachine())
{
    m_get_exo_mode_array_service = create_service<march_shared_msgs::srv::GetExoModeArray>(
        "get_exo_mode_array",
        std::bind(&TestJointsModeMachineNode::handleGetExoModeArray, this, _1, _2));

    m_mode_publisher = create_publisher<march_shared_msgs::msg::ExoModeAndJoint>("current_mode", 10);
    RCLCPP_WARN(rclcpp::get_logger("joint_test_mode_machine"), "Test Joint Mode Machine Node succesfully initialized");
}

TestJointsModeMachineNode::~TestJointsModeMachineNode()
{
    RCLCPP_WARN(rclcpp::get_logger("joint_test_mode_machine"), "Deconstructor of Mode Machine node called");
}

void TestJointsModeMachineNode::fillExoModeArray(march_shared_msgs::srv::GetExoModeArray_Response::SharedPtr response) const
{
    std::set<exoMode> available_modes = m_mode_machine.getAvailableModes((exoMode)m_mode_machine.getCurrentMode());

    // Clear the existing modes in the msg
    response->mode_array.modes.clear();

    // Iterate over the available_modes set and add each mode to the msg
    for (const auto& mode : available_modes) {
        march_shared_msgs::msg::ExoMode exoModeMsg;
        exoModeMsg.mode = static_cast<int8_t>(mode);
        response->mode_array.modes.push_back(exoModeMsg);
    }

    response->current_mode.mode = m_mode_machine.getCurrentMode();
}

void TestJointsModeMachineNode::handleGetExoModeArray(const std::shared_ptr<march_shared_msgs::srv::GetExoModeArray::Request> request,
        std::shared_ptr<march_shared_msgs::srv::GetExoModeArray::Response> response)
{
    RCLCPP_INFO(rclcpp::get_logger("joint_test_mode_machine"), "Request received!");
    exoMode new_mode = (exoMode)request->desired_mode.mode;
    if (m_mode_machine.isValidTransition(new_mode))
    {
        m_mode_machine.performTransition(new_mode);
        auto mode_msg = march_shared_msgs::msg::ExoModeAndJoint();
        mode_msg.mode = m_mode_machine.getCurrentMode();
        mode_msg.joint.data = request->actuated_joint.data;

        m_mode_publisher->publish(mode_msg);
        RCLCPP_INFO(rclcpp::get_logger("mode_machine"), "Publishing current mode!");
    } else 
    {
        RCLCPP_WARN(rclcpp::get_logger("mode_machine"), "Invalid mode transition! Ignoring new mode.");
    }
    fillExoModeArray(response);
    RCLCPP_INFO(rclcpp::get_logger("mode_machine"), "Response sent!");
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestJointsModeMachineNode>());

    rclcpp::shutdown();
    return 0;
}