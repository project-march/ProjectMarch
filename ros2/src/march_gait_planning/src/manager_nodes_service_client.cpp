#include "march_gait_planning/manager_nodes_service_client.hpp"


ServiceClient::ServiceClient()
 : Node("manager_nodes_service_client")
 {
    m_angles_client_get_state = this->create_client<lifecycle_msgs::srv::GetState>(anglesNodeGetStateTopic); 
    m_angles_client_change_state = this->create_client<lifecycle_msgs::srv::ChangeState>(anglesNodeChangeStateTopic); 
    m_cartesian_client_get_state = this->create_client<lifecycle_msgs::srv::GetState>(cartesianNodeGetStateTopic); 
    m_cartesian_client_change_state = this->create_client<lifecycle_msgs::srv::ChangeState>(cartesianNodeChangeStateTopic); 
 }

template <typename FutureT, typename WaitTimeT> std::future_status ServiceClient::waitForResult(FutureT &future, WaitTimeT timeout){
    auto end = std::chrono::steady_clock::now() + timeout; 
    std::chrono::milliseconds wait_period(100); 
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
    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>(); 
    if (!m_angles_client_get_state->wait_for_service(timeout)){
        RCLCPP_ERROR(this->get_logger(), "Service %s not available", m_angles_client_get_state->get_service_name()); 
        return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN; 
    }
    auto futureResult = m_angles_client_get_state->async_send_request(request); 
    auto futureStatus = waitForResult(futureResult, timeout); 
    if (futureStatus != std::future_status::ready){
        RCLCPP_ERROR(this->get_logger(), "Server timed out while getting current state of node %s", anglesNode); 
        return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN; 
    }
    if (futureResult.get()){
        auto state = futureResult.get()->current_state.id; 
        RCLCPP_INFO(this->get_logger(), "Node %s has current state %s", anglesNode, futureResult.get()->current_state.label.c_str()); 
        return state; 
    } 
    else {
        RCLCPP_ERROR(this->get_logger(), "Failed to get current state of node %s", anglesNode); 
    }

}

unsigned int ServiceClient::getCartesianState(std::chrono::seconds timeout){
    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>(); 
    if (!m_cartesian_client_get_state->wait_for_service(timeout)){
        RCLCPP_ERROR(this->get_logger(), "Service %s not available", m_cartesian_client_get_state->get_service_name()); 
        return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN; 
    }
    auto futureResult = m_cartesian_client_get_state->async_send_request(request); 
    auto futureStatus = waitForResult(futureResult, timeout); 
    if (futureStatus != std::future_status::ready){
        RCLCPP_ERROR(this->get_logger(), "Server timed out while getting current state of node %s", cartesianNode); 
        return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN; 
    }
    if (futureResult.get()){
        auto state = futureResult.get()->current_state.id; 
        RCLCPP_INFO(this->get_logger(), "Node %s has current state %s", cartesianNode, futureResult.get()->current_state.label.c_str()); 
        return state; 
    } 
    else {
        RCLCPP_ERROR(this->get_logger(), "Failed to get current state of node %s", cartesianNode); 
    }

}

bool ServiceClient::changeCartesianState(std::uint8_t transition, std::chrono::seconds timeout){
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>(); 
    request->transition.id = transition; 
    if (!m_cartesian_client_change_state->wait_for_service(timeout)){
        RCLCPP_ERROR(this->get_logger(), "Service %s not available", m_cartesian_client_change_state->get_service_name()); 
        return false; 
    }
    auto futureResult = m_cartesian_client_change_state->async_send_request(request); 
    auto futureState = waitForResult(futureResult, timeout); 
    if (futureState != std::future_status::ready){
        RCLCPP_ERROR(this->get_logger(), "Server timed out while getting current state of node %s", cartesianNode); 
        return false; 
    }
    if (futureResult.get()->success){
        RCLCPP_INFO(this->get_logger(), "Transition %d successfully triggered", static_cast<unsigned int>(transition)); 
        return true; 
    }
    else {
        RCLCPP_WARN(this->get_logger(), "Failed to trigger transition %d for node %s", static_cast<unsigned int>(transition), cartesianNode);
        return false; 
    }
}

bool ServiceClient::changeAnglesState(std::uint8_t transition, std::chrono::seconds timeout){
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>(); 
    request->transition.id = transition; 
    if (!m_angles_client_change_state->wait_for_service(timeout)){
        RCLCPP_ERROR(this->get_logger(), "Service %s not available", m_angles_client_change_state->get_service_name()); 
        return false; 
    }
    auto futureResult = m_angles_client_change_state->async_send_request(request); 
    auto futureState = waitForResult(futureResult, timeout); 
    if (futureState != std::future_status::ready){
        RCLCPP_ERROR(this->get_logger(), "Server timed out while getting current state of node %s", anglesNode); 
        return false; 
    }
    if (futureResult.get()->success){
        RCLCPP_INFO(this->get_logger(), "Transition %d successfully triggered", static_cast<unsigned int>(transition)); 
        return true; 
    }
    else {
        RCLCPP_WARN(this->get_logger(), "Failed to trigger transition %d for node %s", static_cast<unsigned int>(transition), anglesNode);
        return false; 
    }
}

void call_script(std::shared_ptr<ServiceClient> service_client)
{
  rclcpp::WallRate stateChangeTime(0.2); //5s, higher Hz = shorter wait time 

  //configure angles and cartesian
  {
    if(!service_client->changeAnglesState(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE) || !service_client->changeCartesianState(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE)){
      return;
    }
    if (!service_client->getAnglesState() || !service_client->getCartesianState()){
      return;
    }
  }



  //activate
  {
    stateChangeTime.sleep();
    if(!service_client->changeAnglesState(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)){
      return;
    }
    if(!service_client->changeCartesianState(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)){
      return;
    }
    if (!service_client->getAnglesState()){
      return;
    }
    if (!service_client->getCartesianState()){
      return;
    }
  }
}
 


int main(int argc, char *argv[]){
    
    rclcpp::init(argc, argv); 

    auto service_client = std::make_shared<ServiceClient>(); 

    rclcpp::executors::SingleThreadedExecutor executor; 
    executor.add_node(service_client); 
    std::shared_future<void> script = std::async(std::launch::async, std::bind(call_script, service_client)); 
    executor.spin_until_future_complete(script); 

    rclcpp::shutdown(); 

    return 0; 
}