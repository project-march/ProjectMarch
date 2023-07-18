#include "weight_shift_buffer/weight_shift_buffer_node.hpp"
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
WeightShiftBufferNode::WeightShiftBufferNode()
    : Node("weight_shift_buffer")
    , m_gait_type(0)
    , m_weight_shift_type(0)
    , m_weight_shift_buffer()
{
    RCLCPP_INFO(this->get_logger(), "Initialized weight shift node");
    this->m_gait_loader_server = rclcpp_action::create_server<control_msgs::action::FollowJointTrajectory>(this,
        "/joint_trajectory_controller/follow_joint_trajectory_buffer",
        std::bind(&WeightShiftBufferNode::handle_goal, this, _1, _2),
        std::bind(&WeightShiftBufferNode::handle_cancel, this, _1),
        std::bind(&WeightShiftBufferNode::handle_accepted, this, _1));

    this->m_joint_controller_client = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
        this, "/joint_trajectory_controller/follow_joint_trajectory");

    m_gait_type_subscriber = this->create_subscription<march_shared_msgs::msg::GaitInstruction>(
        "/march/input_device/instruction", 10, std::bind(&WeightShiftBufferNode::gait_type_callback, this, _1));

    declare_parameter("weight_shift_duration", 0.1);
    declare_parameter("weight_shift_length", 0.0);
    declare_parameter("weight_shift_type", 0);
    declare_parameter("right_swing_scaling", 1.0);
    auto test1 = this->get_parameter("weight_shift_duration").as_double();
    auto test2 = this->get_parameter("weight_shift_length").as_double();
    auto test3 = this->get_parameter("weight_shift_type").as_int();
    auto test4 = this->get_parameter("right_swing_scaling").as_double();
    RCLCPP_INFO(this->get_logger(), "weight shift duration is %f", test1);
    RCLCPP_INFO(this->get_logger(), "weight_shift_length is %f", test2);
    RCLCPP_INFO(this->get_logger(), "weight shift type is %i", test3);
    RCLCPP_INFO(this->get_logger(), "right_swing_leg_scaling is %f", test4);

    RCLCPP_INFO(this->get_logger(), "Initialized weight shift node");
    m_weight_shift_type = test3;
    m_weight_shift_buffer.set_weight_shift_duration(test1);
    m_weight_shift_buffer.set_step_size(test2);
    m_weight_shift_buffer.set_swing_scaling(test4);
}

//
void WeightShiftBufferNode::gait_type_callback(march_shared_msgs::msg::GaitInstruction::SharedPtr msg)
{
    auto test1 = this->get_parameter("weight_shift_duration").as_double();
    auto test2 = this->get_parameter("weight_shift_length").as_double();
    auto test3 = this->get_parameter("weight_shift_type").as_int();
    auto test4 = this->get_parameter("right_swing_scaling").as_double();
    RCLCPP_INFO(this->get_logger(), "weight shift duration is %f", test1);
    RCLCPP_INFO(this->get_logger(), "weight_shift_length is %f", test2);
    RCLCPP_INFO(this->get_logger(), "weight shift type is %i", test3);
    RCLCPP_INFO(this->get_logger(), "right_swing_leg_scaling is %f", test4);
    m_weight_shift_type = test3;
    m_weight_shift_buffer.set_weight_shift_duration(test1);
    m_weight_shift_buffer.set_step_size(test2);
    m_weight_shift_buffer.set_swing_scaling(test4);

    m_gait_type = 0;
    if (msg->gait_name.compare("fixed_walk") == 0 || msg->gait_name.compare("fixed_step_and_close") == 0) {
        RCLCPP_WARN(this->get_logger(), "Received walk goal");
        m_weight_shift_buffer.swing_leg = "R";
        m_gait_type = 1;
    }
}

// Action server stuff(interacts with gait loader)
rclcpp_action::GoalResponse WeightShiftBufferNode::handle_goal(
    const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal)
{
    (void)uuid;
    RCLCPP_INFO(this->get_logger(), "Received goal");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse WeightShiftBufferNode::handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle)
{
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void WeightShiftBufferNode::handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Accept request from gait loader");
    std::thread { std::bind(&WeightShiftBufferNode::execute, this, _1), goal_handle }.detach();
}

void WeightShiftBufferNode::execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle)
{
    m_server_goal_handle = goal_handle;
    // Receive and execute
    auto goal = goal_handle->get_goal();
    request_feedback(*goal);
}
// Action client stuf(interacts with joint trajectory controller)
void WeightShiftBufferNode::request_feedback(control_msgs::action::FollowJointTrajectory::Goal goal)
{
    RCLCPP_INFO(this->get_logger(), "Requesting feedback");
    if (!this->m_joint_controller_client->wait_for_action_server()) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        rclcpp::shutdown();
    }
    RCLCPP_INFO(this->get_logger(), "Action server is alive");
    auto goal_msg = goal;
    if (m_gait_type == 1) {
        RCLCPP_ERROR(this->get_logger(), "I am walking");
        switch(m_weight_shift_type)
        {
            case 0:
            //Do nothing
            break;

            case 1:
            // Make right step more extreme than left
            goal_msg.trajectory = m_weight_shift_buffer.fix_asymmetry(goal.trajectory);
            break;

            case 2:
            // Add weight shift during step
            m_weight_shift_buffer.set_incoming_joint_trajectory(goal.trajectory);
            goal_msg.trajectory = m_weight_shift_buffer.add_weight_shift_during_gait();
            break;

            case 3:
            // Add weight shift inbetween steps
            goal_msg.trajectory = m_weight_shift_buffer.return_final_traj_with_weight_shift(goal.trajectory);
            break;
        }
    }

    auto send_goal_options = rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&WeightShiftBufferNode::goal_response_callback, this, _1);
    send_goal_options.feedback_callback = std::bind(&WeightShiftBufferNode::feedback_callback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&WeightShiftBufferNode::result_callback, this, _1);
    RCLCPP_INFO(this->get_logger(), "bound callback functions");
    this->m_joint_controller_client->async_send_goal(goal_msg, send_goal_options);
}

void WeightShiftBufferNode::goal_response_callback(
    std::shared_future<rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr> future)
{
    auto goal_handle = future.get();
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
}

void WeightShiftBufferNode::feedback_callback(
    rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr,
    const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Feedback> feedback)
{
    // RCLCPP_INFO(this->get_logger(), "Received feedback");
    std::shared_ptr<control_msgs::action::FollowJointTrajectory::Feedback> nonconst_feedback
        = std::make_shared<control_msgs::action::FollowJointTrajectory::Feedback>();
    *nonconst_feedback = *feedback;
    if (m_server_goal_handle) {
        m_server_goal_handle->publish_feedback(nonconst_feedback);
    }
    // RCLCPP_INFO(this->get_logger(), "Accepted feedback");
}

void WeightShiftBufferNode::result_callback(
    const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::WrappedResult& result)
{
    RCLCPP_WARN(this->get_logger(), "Result callback called");
    if (m_server_goal_handle) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal was succesful");
                m_server_goal_handle->succeed(result.result);
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                m_server_goal_handle->abort(result.result);
                return;
            case rclcpp_action::ResultCode::CANCELED:
                if (m_server_goal_handle->is_canceling()) {
                    RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                    m_server_goal_handle->canceled(result.result);
                }
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                return;
        }
    }
}
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WeightShiftBufferNode>());
    rclcpp::shutdown();
    return 0;
}
