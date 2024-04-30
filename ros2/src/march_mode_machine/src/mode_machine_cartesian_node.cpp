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

    // This is a dummy publisher to send planes to the footstep planner (in absence of camera module)
    m_planes_dummy_publisher = create_publisher<march_shared_msgs::msg::AllPlanes>("planes", 100); 

}

ModeMachineCartesianNode::~ModeMachineCartesianNode()
{
    RCLCPP_WARN(this->get_logger(), "Deconstructor of Mode Machine node called");
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
        
        // In case of VariableStep, send a dummy plane (as if it came from cams) to gaitplanning. 
        if (mode_msg.mode == 10){
            march_shared_msgs::msg::AllPlanes plane_msg; 
            march_shared_msgs::msg::Plane plane; 
            plane.centroid.x = 0.7; 
            plane.centroid.y = 0.16; 
            plane.centroid.z = 0.0;
            plane.left_boundary_point.y = 0.6; 
            plane.right_boundary_point.y = -0.6; 
            plane.upper_boundary_point.x = 1.2; 
            plane.lower_boundary_point.x = 0.0; 
            plane_msg.planes = {plane}; 
            m_planes_dummy_publisher->publish(plane_msg); 
            RCLCPP_INFO(this->get_logger(), "Planes sent!"); 
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