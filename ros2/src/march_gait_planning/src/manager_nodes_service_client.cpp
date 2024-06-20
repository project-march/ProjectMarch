#include "march_gait_planning/manager_nodes_service_client.hpp"

#define COLOR_GREEN   "\033[32m"
#define RESET   "\033[0m"

ServiceClient::ServiceClient()
 : Node("manager_nodes_service_client")
 {
    m_angles_client_get_state = this->create_client<lifecycle_msgs::srv::GetState>(anglesNodeGetStateTopic); 
    m_angles_client_change_state = this->create_client<lifecycle_msgs::srv::ChangeState>(anglesNodeChangeStateTopic); 
    m_cartesian_client_get_state = this->create_client<lifecycle_msgs::srv::GetState>(cartesianNodeGetStateTopic); 
    m_cartesian_client_change_state = this->create_client<lifecycle_msgs::srv::ChangeState>(cartesianNodeChangeStateTopic); 

    m_mode_subscriber = this->create_subscription<march_shared_msgs::msg::ExoMode>(
        "current_mode", 10, std::bind(&ServiceClient::modeCallback, this, std::placeholders::_1));  

    m_gaitplanning_mode_publisher = this->create_publisher<march_shared_msgs::msg::ExoMode>("gait_planning_mode", 10);

    m_angles_active = true; 
    m_cartesian_active = true; 
}

void ServiceClient::publishModeToGaitPlanning(const march_shared_msgs::msg::ExoMode::SharedPtr msg){
    auto mode_msg = march_shared_msgs::msg::ExoMode();
    mode_msg.mode = msg->mode; 
    mode_msg.node_type = msg->node_type; 
    m_gaitplanning_mode_publisher->publish(mode_msg);
}

void ServiceClient::modeCallback(const march_shared_msgs::msg::ExoMode::SharedPtr msg){
    RCLCPP_INFO(this->get_logger(), "Received new mode! %s \n", toString(static_cast<ExoMode>(msg->mode)).c_str()); 
    if ((ExoMode)msg->mode == ExoMode::Stand){
        if (m_angles_active == true && m_cartesian_active == true){
            changeCartesianState(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE); 
            // changeAnglesState(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
            m_angles_active = true; 
            m_cartesian_active = false; 
            publishModeToGaitPlanning(msg); 
        } else if (m_angles_active == true && m_cartesian_active == false){
            RCLCPP_INFO(this->get_logger(), "Letting the gait finish"); 
            publishModeToGaitPlanning(msg); 
        } else if (m_angles_active == false && m_cartesian_active == true){
            RCLCPP_INFO(this->get_logger(), "Letting the gait finish"); 
            publishModeToGaitPlanning(msg); 
        } else {
            RCLCPP_ERROR(this->get_logger(), "Both gait planning nodes inactive!"); 
        }
    } else if (msg->node_type == "joint_angles" && (ExoMode)msg->mode != ExoMode::Stand){
        if (!m_angles_active) changeAnglesState(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
        if (m_cartesian_active) changeCartesianState(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);   
        m_angles_active = true; 
        m_cartesian_active = false; 
        publishModeToGaitPlanning(msg); 
    } else if (msg->node_type == "cartesian" && (ExoMode)msg->mode != ExoMode::Stand){
        if (!m_cartesian_active) changeCartesianState(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);  
        if (m_angles_active) changeAnglesState(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE); 
        m_angles_active = false;
        m_cartesian_active = true; 
        publishModeToGaitPlanning(msg); 
    }
    else {
        RCLCPP_WARN(this->get_logger(), "Unknown node type: %s", msg->node_type.c_str()); 
    }
}

template <typename FutureT, typename WaitTimeT> std::future_status ServiceClient::waitForResult(FutureT &future, WaitTimeT timeout){
    auto end = std::chrono::steady_clock::now() + timeout; 
    std::chrono::milliseconds wait_period(50); 
    std::future_status status = std::future_status::timeout; 
    do {
        auto now = std::chrono::steady_clock::now(); 
        auto time_left = end - now;
        if (time_left <= std::chrono::seconds(0)){
            break; 
        }
        status = future.wait_for((time_left < wait_period)? time_left: wait_period); 
    }
    while (rclcpp::ok() && status != std::future_status::ready); 
    return status; 
}

unsigned int ServiceClient::getAnglesState(std::chrono::seconds timeout){
    RCLCPP_WARN(this->get_logger(), "getAnglesState() called"); 
    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>(); 

    if (!m_angles_client_get_state->wait_for_service(timeout)){
        RCLCPP_ERROR(this->get_logger(), "Service %s not available \n", m_angles_client_get_state->get_service_name()); 
        return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN; 
    }
    auto futureResult = m_angles_client_get_state->async_send_request(request); 
    auto futureStatus = waitForResult(futureResult, timeout); 
    if (futureStatus != std::future_status::ready){
        RCLCPP_ERROR(this->get_logger(), "Server timed out while getting current state of node %s \n", anglesNode); 
        return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN; 
    }
    if (futureResult.get()){
        auto state = futureResult.get()->current_state.id; 
        RCLCPP_WARN(this->get_logger(), "Node %s has current state %s \n", anglesNode, futureResult.get()->current_state.label.c_str()); 
        return state; 
    } 
    else {
        RCLCPP_ERROR(this->get_logger(), "Failed to get current state of node %s \n", anglesNode); 
        return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

}

unsigned int ServiceClient::getCartesianState(std::chrono::seconds timeout){
    RCLCPP_WARN(this->get_logger(), "getCartesianState() called"); 
    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>(); 
    if (!m_cartesian_client_get_state->wait_for_service(timeout)){
        RCLCPP_ERROR(this->get_logger(), "Service %s not available \n", m_cartesian_client_get_state->get_service_name()); 
        return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN; 
    }
    auto futureResult = m_cartesian_client_get_state->async_send_request(request); 
    auto futureStatus = waitForResult(futureResult, timeout); 
    if (futureStatus != std::future_status::ready){
        RCLCPP_ERROR(this->get_logger(), "Server timed out while getting current state of node %s \n", cartesianNode); 
        return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN; 
    }
    if (futureResult.get()){
        auto state = futureResult.get()->current_state.id; 
        RCLCPP_WARN(this->get_logger(), "Node %s has current state %s \n", cartesianNode, futureResult.get()->current_state.label.c_str()); 
        return state; 
    } 
    else {
        RCLCPP_ERROR(this->get_logger(), "Failed to get current state of node %s \n", cartesianNode); 
        return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

}

bool ServiceClient::changeCartesianState(std::uint8_t transition, std::chrono::seconds timeout){
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>(); 
    request->transition.id = transition; 
    if (!m_cartesian_client_change_state->wait_for_service(timeout)){
        RCLCPP_ERROR(this->get_logger(), "Service %s not available \n", m_cartesian_client_change_state->get_service_name()); 
        return false; 
    }
    auto futureResult = m_cartesian_client_change_state->async_send_request(request);
    futureResult.wait_for(timeout); 
    return true;
}

bool ServiceClient::changeAnglesState(std::uint8_t transition, std::chrono::seconds timeout){
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>(); 
    request->transition.id = transition; 
    if (!m_angles_client_change_state->wait_for_service(timeout)){
        RCLCPP_ERROR(this->get_logger(), "Service %s not available \n", m_angles_client_change_state->get_service_name()); 
        return false; 
    }
    auto futureResult = m_angles_client_change_state->async_send_request(request);
    futureResult.wait_for(timeout); 
    return true; 
}

void call_script(std::shared_ptr<ServiceClient> service_client)
{
    rclcpp::WallRate stateChangeTime(0.2); //5s, higher Hz = shorter wait time 

    if (!service_client->changeAnglesState(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE)){
        return; 
    }
    if (!service_client->changeCartesianState(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE)){
        return; 
    }

    stateChangeTime.sleep(); 
    if (!service_client->changeAnglesState(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)){
        return; 
    }
    if (!service_client->changeCartesianState(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)){
        return; 
    } 

    RCLCPP_INFO(service_client->get_logger(), COLOR_GREEN "Nodes activated! Ready to gait." RESET); 
}
 
int main(int argc, char *argv[]){
    
    rclcpp::init(argc, argv); 

    auto service_client = std::make_shared<ServiceClient>(); 

    rclcpp::executors::SingleThreadedExecutor executor; 
    executor.add_node(service_client); 
    std::shared_future<void> script = std::async(std::launch::async, std::bind(call_script, service_client)); 
    executor.spin(); 

    rclcpp::shutdown(); 

    return 0; 
}