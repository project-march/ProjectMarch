#include "march_gait_planning/gait_planning_node.hpp"

using std::placeholders::_1; 

GaitPlanningNode::GaitPlanningNode()
 : Node("march_gait_planning_node"), 
   m_gait_planning(GaitPlanning())
 {
    std::cout << "gait planning node created" << std::endl; 
    m_iks_foot_positions_publisher = create_publisher<march_shared_msgs::msg::IksFootPositions>("iks_foot_positions", 10);
    std::cout << "iks foot positions publisher created" << std::endl; 
    m_exo_state_subscriber = create_subscription<march_shared_msgs::msg::ExoState>(
        "current_state", 10, std::bind(&GaitPlanningNode::currentStateCallback, this, _1));
    std::cout << "exo state subscriber created" << std::endl; 
    m_current_stance_foot_subscriber = create_subscription<std_msgs::msg::Int32>(
        "current_stance_foot", 10, std::bind(&GaitPlanningNode::currentStanceFootCallback, this, _1)); 
    std::cout << "current stance foot subscriber created" << std::endl; 
    m_feet_position_subscriber = create_subscription<march_shared_msgs::msg::IksFootPositions>(
        "estimated_baseframe_foot_positions", 100, std::bind(&GaitPlanningNode::currentFeetPositionsCallback, this, _1)); 
    std::cout << "current baseframe foot positions subscriber created" << std::endl; 

    m_stance_leg_request = std::make_shared<march_shared_msgs::srv::GetCurrentStanceLeg::Request>();
    m_stance_leg_client = create_client<march_shared_msgs::srv::GetCurrentStanceLeg>("current_stance_leg_service");
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
    
    if (response){
        RCLCPP_INFO(rclcpp::get_logger("march_gait_selection"), "Stance leg request received successful!");
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("march_gait_selection"), "Stance leg request was not successful!"); 
    }
}

void GaitPlanningNode::sendRequest(const bool& gait_complete){
    if (gait_complete){
        m_stance_leg_request->gait_complete = gait_complete; 
        auto future = m_stance_leg_client->async_send_request(m_stance_leg_request, std::bind(&GaitPlanningNode::responseStanceLegCallback, this, _1)); 
    } else {
        return; 
    }
}

void GaitPlanningNode::footPositionsPublish(){

}

//TODO: create function that adds y values to foot coordinates, based on stance foot. 
// if stance leg is both, take right foot to complete initial 10cm step first. 
// if stance leg is either left or right, take the other foot to complete the 20cm step. 

//TODO: create function that publishes the bezier curve points per time point. 


int main(int argc, char *argv[]){
    
    rclcpp::init(argc, argv); 

    rclcpp::spin(std::make_shared<GaitPlanningNode>()); 
    rclcpp::shutdown(); 

    return 0; 
}