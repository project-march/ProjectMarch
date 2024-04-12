#include "march_gait_planning/manager_nodes_service_client.hpp"


ServiceClient::ServiceClient()
 : Node("manager_nodes_service_client")
 {
    m_client_get_state = this->create_client<lifecycle_msgs::srv::GetState>(nodeGetStateTopic); 
    m_client_change_state = this->create_client<lifecycle_msgs::srv::ChangeState>(nodeChangeStateTopic); 
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

unsigned int ServiceClient::getState(std::chrono::seconds timeout){
    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>(); 
    if (!m_client_get_state->wait_for_service(timeout)){
        RCLCPP_ERROR(this->get_logger(), "Service %s not available", m_client_get_state->get_service_name()); 
        return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN; 
    }
    auto futureResult = m_client_get_state->async_send_request(request); 
    auto futureStatus = waitForResult(futureResult, timeout); 
    if (futureStatus != std::future_status::ready){
        RCLCPP_ERROR(this->get_logger(), "Server timed out while getting current state of node %s", talkerNode); 
        return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN; 
    }
    if (futureResult.get()){
        auto state = futureResult.get()->current_state.id; 
        RCLCPP_INFO(this->get_logger(), "Node %s has current state %s", talkerNode, futureResult.get()->current_state.label.c_str()); 
        return state; 
    } 
    else {
        RCLCPP_ERROR(this->get_logger(), "Failed to get current state of node %s", talkerNode); 
    }

}

bool ServiceClient::changeState(std::uint8_t transition, std::chrono::seconds timeout){
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>(); 
    request->transition.id = transition; 
    if (!m_client_change_state->wait_for_service(timeout)){
        RCLCPP_ERROR(this->get_logger(), "Service %s not available", m_client_change_state->get_service_name()); 
        return false; 
    }
    auto futureResult = m_client_change_state->async_send_request(request); 
    auto futureState = waitForResult(futureResult, timeout); 
    if (futureState != std::future_status::ready){
        RCLCPP_ERROR(this->get_logger(), "Server timed out while getting current state of node %s", talkerNode); 
        return false; 
    }
    if (futureResult.get()->success){
        RCLCPP_INFO(this->get_logger(), "Transition %d successfully triggered", static_cast<unsigned int>(transition)); 
        return true; 
    }
    else {
        RCLCPP_WARN(this->get_logger(), "Failed to trigger transition %d for node %s", static_cast<unsigned int>(transition), talkerNode);
        return false; 
    }
}

void call_script(std::shared_ptr<ServiceClient> service_client)
{
  rclcpp::WallRate stateChangeTime(0.2); //5s, higher Hz = shorter wait time 

  //configure
  {
    if(!service_client->changeState(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE)){
      return;
    }
    if (!service_client->getState()){
      return;
    }
  }

  //activate
  {
    stateChangeTime.sleep();
    if(!service_client->changeState(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)){
      return;
    }
    if (!service_client->getState()){
      return;
    }
  }

  // //deactivate
  // {
  //   stateChangeTime.sleep();
  //   if(!service_client->changeState(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE)){
  //     return;
  //   }
  //   if (!service_client->getState()){
  //     return;
  //   }
  // }

  // //activate
  // {
  //   stateChangeTime.sleep();
  //   if(!service_client->changeState(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)){
  //     return;
  //   }
  //   if (!service_client->getState()){
  //     return;
  //   }
  // }

  // //deactivate
  // {
  //   stateChangeTime.sleep();
  //   if(!service_client->changeState(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE)){
  //     return;
  //   }
  //   if (!service_client->getState()){
  //     return;
  //   }
  // }

  // //cleanup
  // {
  //   stateChangeTime.sleep();
  //   if(!service_client->changeState(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP)){
  //     return;
  //   }
  //   if (!service_client->getState()){
  //     return;
  //   }
  // }

  // //unconfigured shutdown
  // {
  //   stateChangeTime.sleep();
  //   if(!service_client->changeState(lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN)){
  //     return;
  //   }
  //   if (!service_client->getState()){
  //     return;
  //   }
  // }

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