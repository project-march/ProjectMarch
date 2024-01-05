/*Authors: Andrew Hutani, MIX

This node is a seperate State Machine for the testing of single joints. It only uses states BootUp, Stand, and Walk, where BootUp is used to switch joints and 
Walk to send a sinusoidal wave to the joint.
This node will send the actuated joint alongside the state to the gait planning node.

This node is only called in the test_joints launch file.

*/

#include "state_machine/test_joints_state_machine_node.hpp"
using std::placeholders::_1;
using std::placeholders::_2;


TestJointsStateMachineNode::TestJointsStateMachineNode()
:   Node("state_machine_node"),
    m_state_machine(TestJointsStateMachine())
{
    m_get_exo_state_array_service = create_service<march_shared_msgs::srv::GetExoStateArray>(
        "get_exo_state_array",
        std::bind(&TestJointsStateMachineNode::handleGetExoStateArray, this, _1, _2));

    m_state_publisher = create_publisher<march_shared_msgs::msg::ExoStateAndJoint>("current_state", 10);
    RCLCPP_WARN(rclcpp::get_logger("joint_test_state_machine"), "Joint State Machine Node succesfully initialized");
}

TestJointsStateMachineNode::~TestJointsStateMachineNode()
{
    RCLCPP_WARN(rclcpp::get_logger("joint_test_state_machine"), "Deconstructor of State Machine node called");
}

void TestJointsStateMachineNode::fillExoStateArray(march_shared_msgs::srv::GetExoStateArray_Response::SharedPtr response) const
{
    std::set<exoState> available_states = m_state_machine.getAvailableStates((exoState)m_state_machine.getCurrentState());

    // Clear the existing states in the msg
    response->state_array.states.clear();

    // Iterate over the available_states set and add each state to the msg
    for (const auto& state : available_states) {
        march_shared_msgs::msg::ExoState exoStateMsg;
        exoStateMsg.state = static_cast<int8_t>(state);
        response->state_array.states.push_back(exoStateMsg);
    }

    response->current_state.state = m_state_machine.getCurrentState();
}

void TestJointsStateMachineNode::handleGetExoStateArray(const std::shared_ptr<march_shared_msgs::srv::GetExoStateArray::Request> request,
        std::shared_ptr<march_shared_msgs::srv::GetExoStateArray::Response> response)
{
    RCLCPP_INFO(rclcpp::get_logger("joint_test_state_machine"), "Request received!");
    exoState new_state = (exoState)request->desired_state.state;
    if (m_state_machine.isValidTransition(new_state))
    {
        m_state_machine.performTransition(new_state);
        auto state_msg = march_shared_msgs::msg::ExoStateAndJoint();
        state_msg.state = m_state_machine.getCurrentState();
        state_msg.joint.data = request->actuated_joint.data;

        m_state_publisher->publish(state_msg);
    } else 
    {
        RCLCPP_WARN(rclcpp::get_logger("state_machine"), "Invalid state transition! Ignoring new state.");
    }
    fillExoStateArray(response);
    RCLCPP_INFO(rclcpp::get_logger("state_machine"), "Response sent!");
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestJointsStateMachineNode>());

    rclcpp::shutdown();
    return 0;
}