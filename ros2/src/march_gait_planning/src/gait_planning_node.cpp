/*Authors: Femke Buiks and Andrew Hutani, MIX*/

#include "march_gait_planning/gait_planning_node.hpp"

using std::placeholders::_1; 

GaitPlanningNode::GaitPlanningNode()
 : Node("march_gait_planning_node"), 
   m_gait_planning(GaitPlanning()),
   m_current_trajectory(),
   m_current_step_msg(std::make_shared<march_shared_msgs::msg::IksFootPositions>())
 {
    m_iks_foot_positions_publisher = create_publisher<march_shared_msgs::msg::IksFootPositions>("iks_foot_positions", 10);
    m_exo_state_subscriber = create_subscription<march_shared_msgs::msg::ExoState>(
        "current_state", 10, std::bind(&GaitPlanningNode::currentStateCallback, this, _1));
    // m_current_stance_foot_subscriber = create_subscription<std_msgs::msg::Int32>(
        // "current_stance_foot", 10, std::bind(&GaitPlanningNode::currentStanceFootCallback, this, _1)); 
    m_feet_position_subscriber = create_subscription<march_shared_msgs::msg::IksFootPositions>(
        "estimated_baseframe_foot_positions", 100, std::bind(&GaitPlanningNode::currentFeetPositionsCallback, this, _1)); 

    m_stance_leg_request = std::make_shared<march_shared_msgs::srv::GetCurrentStanceLeg::Request>();
    m_stance_leg_client = create_client<march_shared_msgs::srv::GetCurrentStanceLeg>("current_stance_leg_service");
    std::cout << "Request and client created " << std::endl; 

    // If everything goes correctly, there is nothing to publish so immediately a request will be sent. 
    footPositionsPublish(); 
 }

void GaitPlanningNode::currentStateCallback(const march_shared_msgs::msg::ExoState::SharedPtr msg){
    RCLCPP_INFO(get_logger(), "Received current state: %d", msg->state); 
    m_gait_planning.setGaitType((exoState)msg->state); 
}

void GaitPlanningNode::currentStanceFootCallback(const std_msgs::msg::Int32::SharedPtr msg){
    RCLCPP_INFO(get_logger(), "Received current stance foot: %d", msg->data); 
    m_gait_planning.setStanceFoot(msg->data); 
}

void GaitPlanningNode::currentFeetPositionsCallback(const march_shared_msgs::msg::IksFootPositions::SharedPtr msg){
    RCLCPP_INFO(get_logger(), "Received current foot positions");
    std::array<double, 3> new_left_foot_position = {msg->left_foot_position.x, msg->left_foot_position.y, msg->left_foot_position.z};
    std::array<double, 3> new_right_foot_position = {msg->right_foot_position.x, msg->right_foot_position.y, msg->right_foot_position.z};
    m_gait_planning.setFootPositions(new_left_foot_position, new_right_foot_position); 
}

void GaitPlanningNode::responseStanceLegCallback(
    std::shared_future<march_shared_msgs::srv::GetCurrentStanceLeg::Response::SharedPtr> future){
    // Get the response from the future
    auto response = future.get();

    RCLCPP_INFO(rclcpp::get_logger("march_gait_planning"), std::to_string(response->stance_leg) + " is the stance leg");

    if (response){
        RCLCPP_INFO(rclcpp::get_logger("march_gait_planning"), "Stance leg response received successful!");
        m_gait_planning.setStanceFoot(response->stance_leg);
        m_current_trajectory = m_gait_planning.getTrajectory(); 
        RCLCPP_INFO(rclcpp::get_logger("march_gait_planning"), "Trajectory size = %d", m_current_trajectory.size());
        footPositionsPublish();
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("march_gait_planning"), "Stance leg response was not successful!"); 
    }
}

void GaitPlanningNode::sendRequest(const bool& gait_complete){
    if (gait_complete){
        if (!m_stance_leg_client->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Service not available after waiting");
            if (rclcpp::ok()) {
                sendRequest(gait_complete);
            }
        }
        m_stance_leg_request->gait_complete = gait_complete; 
        auto future = m_stance_leg_client->async_send_request(m_stance_leg_request, std::bind(&GaitPlanningNode::responseStanceLegCallback, this, _1)); 
        RCLCPP_INFO(rclcpp::get_logger("march_gait_planning"), "Request sent!"); 
    } else {
        return; 
    }
}

void GaitPlanningNode::footPositionsPublish(){
    while (true) {
        RCLCPP_INFO(rclcpp::get_logger("march_gait_planning"), "empty yes no %d", m_current_trajectory.empty());
        if (m_current_trajectory.empty()) {
            sendRequest(true);
            break;
        }
        std::array<double, 4> current_step = m_current_trajectory.front();
        m_current_trajectory.erase(m_current_trajectory.begin());
        RCLCPP_INFO(rclcpp::get_logger("march_gait_planning"), "Current stance foot is= %d", m_gait_planning.getCurrentStanceFoot());
        if (m_gait_planning.getCurrentStanceFoot() == -1 || m_gait_planning.getCurrentStanceFoot() == 0){
            RCLCPP_INFO(rclcpp::get_logger("march_gait_planning"), "stance foot = left");
            m_current_step_msg->left_foot_position.x = current_step[2];
            m_current_step_msg->left_foot_position.y = m_gait_planning.getCurrentLeftFootPos()[1];
            m_current_step_msg->left_foot_position.z = current_step[3]; 
            m_current_step_msg->right_foot_position.x = current_step[0]; 
            m_current_step_msg->right_foot_position.y = m_gait_planning.getCurrentRightFootPos()[1];
            m_current_step_msg->right_foot_position.z = current_step[1];
        } else if (m_gait_planning.getCurrentStanceFoot() == 1){
            RCLCPP_INFO(rclcpp::get_logger("march_gait_planning"), "stance foot = right");
            m_current_step_msg->left_foot_position.x = current_step[0];
            m_current_step_msg->left_foot_position.y = m_gait_planning.getCurrentLeftFootPos()[1];
            m_current_step_msg->left_foot_position.z = current_step[1]; 
            m_current_step_msg->right_foot_position.x = current_step[2]; 
            m_current_step_msg->right_foot_position.y = m_gait_planning.getCurrentRightFootPos()[1];
            m_current_step_msg->right_foot_position.z = current_step[3];
        }
        RCLCPP_INFO(rclcpp::get_logger("march_gait_planning"), "exited loop");
        m_iks_foot_positions_publisher->publish(*m_current_step_msg);
        RCLCPP_INFO(rclcpp::get_logger("march_gait_planning"), "Msg published! Length of array = %d", m_current_trajectory.size()); 
        rclcpp::sleep_for(std::chrono::milliseconds(50));
    }
}

int main(int argc, char *argv[]){
    
    rclcpp::init(argc, argv); 

    rclcpp::spin(std::make_shared<GaitPlanningNode>()); 
    rclcpp::shutdown(); 

    return 0; 
}