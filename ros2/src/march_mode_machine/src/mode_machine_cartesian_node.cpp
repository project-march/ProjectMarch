#include "march_mode_machine/mode_machine_cartesian_node.hpp"
using std::placeholders::_1;
using std::placeholders::_2;

ModeMachineCartesianNode::ModeMachineCartesianNode()
: Node("mode_machine_node"), 
  m_mode_machine(ModeMachineCartesian())
{
    m_get_exo_mode_array_service = create_service<march_shared_msgs::srv::GetExoModeArray>(
        "get_exo_mode_array",
        std::bind(&ModeMachineCartesianNode::handleGetExoModeArray, this, _1, _2));

    m_mode_publisher = create_publisher<march_shared_msgs::msg::ExoMode>("current_mode", 10);
    RCLCPP_WARN(this->get_logger(), "Cartesian Mode Machine Node succesfully initialized");

    //TODO: This publisher should not be here, this information should come from the footstepplanner. We do not have this module as of yet so this is a replacement mock. 
    m_footsteps_dummy_publisher = create_publisher<march_shared_msgs::msg::FootStepOutput>("footsteps", 100); 

    // Restore service client relation with footstep planner 
    m_footstep_client = this->create_client<march_shared_msgs::srv::RequestFootsteps>("footstep_generator");
    m_footstep_request = std::make_shared<march_shared_msgs::srv::RequestFootsteps::Request>();

    while (!m_footstep_client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "footstep_planner service not available, waiting again...");
    }
    RCLCPP_INFO(this->get_logger(), "footstep_planner service connected");

}

ModeMachineCartesianNode::~ModeMachineCartesianNode()
{
    RCLCPP_WARN(this->get_logger(), "Deconstructor of Mode Machine node called");
}

void ModeMachineCartesianNode::sendRequest(const int& desired_mode){
    m_footstep_request->gait_type = desired_mode;
    m_footstep_request->stance_leg = 0; 
    m_footstep_future = m_footstep_client->async_send_request(
        m_footstep_request, std::bind(&ModeMachineCartesianNode::responseFootstepCallback, this, _1));

}

void ModeMachineCartesianNode::responseFootstepCallback(
    const rclcpp::Client<march_shared_msgs::srv::RequestFootsteps>::SharedFuture future)
{
    if (future.get()->status) {
        RCLCPP_INFO(this->get_logger(), "Footstep request received successful!");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Footstep request was not a success!");
    }
}


void ModeMachineCartesianNode::fillExoModeArray(march_shared_msgs::srv::GetExoModeArray_Response::SharedPtr response) const
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

void ModeMachineCartesianNode::handleGetExoModeArray(const std::shared_ptr<march_shared_msgs::srv::GetExoModeArray::Request> request,
        std::shared_ptr<march_shared_msgs::srv::GetExoModeArray::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Request received!");
    exoMode new_mode = (exoMode)request->desired_mode.mode;
    if (m_mode_machine.isValidTransition(new_mode))
    {
        m_mode_machine.performTransition(new_mode);
        auto mode_msg = march_shared_msgs::msg::ExoMode();
        mode_msg.mode = m_mode_machine.getCurrentMode();
        m_mode_publisher->publish(mode_msg);
        
        if (m_mode_machine.getCurrentMode() == 10){
            sendRequest(2); 
        }

        //TODO: Remove following logic from here. When you now activate the VariableWalk, the mode machine will send a distance of 0.4 to the gait planning module. This distance should come from footstepplanner. Somewhere
        // the logic to identify when we use the cameras should be included (maybe a boolean?)
        if (mode_msg.mode == 10){
            march_shared_msgs::msg::FootStepOutput feet_msg; 
            feet_msg.distance = 0.4;
            m_footsteps_dummy_publisher->publish(feet_msg); 
        }

    } else 
    {
        RCLCPP_WARN(this->get_logger(), "Invalid mode transition! Ignoring new mode.");
    }
    fillExoModeArray(response);
    RCLCPP_INFO(this->get_logger(), "Response sent!");
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ModeMachineCartesianNode>());

    rclcpp::shutdown();
    return 0;
}