/*Authors: Femke Buiks and Andrew Hutani, MIX*/

#include "march_gait_planning/gait_planning_node.hpp"
#include "../../state_machine/include/state_machine/exo_state.hpp"

using std::placeholders::_1; 

GaitPlanningNode::GaitPlanningNode()
 : Node("march_gait_planning_node"), 
   m_gait_planning(GaitPlanning()),
   m_desired_footpositions_msg(std::make_shared<march_shared_msgs::msg::IksFootPositions>()),
   m_response_received(true)
 {
    m_iks_foot_positions_publisher = create_publisher<march_shared_msgs::msg::IksFootPositions>("iks_foot_positions", 10);

    m_exo_state_subscriber = create_subscription<march_shared_msgs::msg::ExoState>(
        "current_state", 10, std::bind(&GaitPlanningNode::currentStateCallback, this, _1)); 
    m_feet_position_subscriber = create_subscription<march_shared_msgs::msg::IksFootPositions>(
        "estimated_baseframe_foot_positions", 100, std::bind(&GaitPlanningNode::currentFeetPositionsCallback, this, _1)); 

    m_stance_leg_request = std::make_shared<march_shared_msgs::srv::GetCurrentStanceLeg::Request>();
    m_stance_leg_client = create_client<march_shared_msgs::srv::GetCurrentStanceLeg>("current_stance_leg_service");
    std::cout << "Request and client created " << std::endl; 

    m_gait_planning.setGaitType(exoState::BootUp); 

    auto timer_callback = std::bind(&GaitPlanningNode::timerCallback, this);
    m_timer = this->create_wall_timer(std::chrono::milliseconds(50), timer_callback);
 }

void GaitPlanningNode::currentStateCallback(const march_shared_msgs::msg::ExoState::SharedPtr msg){
    RCLCPP_INFO(get_logger(), "Received current state: %d", msg->state); 
    m_gait_planning.setGaitType((exoState)msg->state);
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
        m_gait_planning.setStanceFoot(response->stance_leg);
        m_current_trajectory = m_gait_planning.getTrajectory(); 
        m_response_received = true;
        RCLCPP_INFO(rclcpp::get_logger("march_gait_planning"), "Response received!");
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("march_gait_planning"), "Stance leg response was not successful!"); 
    }
}

void GaitPlanningNode::sendRequest(const bool& gait_complete){
    if (gait_complete){
        m_response_received = false;
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

void GaitPlanningNode::setFootPositions(double left_x, double left_y, double left_z, 
                                        double right_x, double right_y, double right_z) 
{
    m_desired_footpositions_msg->left_foot_position.x = left_x;
    m_desired_footpositions_msg->left_foot_position.y = left_y;
    m_desired_footpositions_msg->left_foot_position.z = left_z; 
    m_desired_footpositions_msg->right_foot_position.x = right_x; 
    m_desired_footpositions_msg->right_foot_position.y = right_y;
    m_desired_footpositions_msg->right_foot_position.z = right_z;
}

void GaitPlanningNode::footPositionsPublish(){
    switch (m_gait_planning.getGaitType()){
        case exoState::Stand :
            m_current_trajectory.clear();
            setFootPositions(0.0, 0.16, -0.802, 0.0, -0.16, -0.802);
            m_iks_foot_positions_publisher->publish(*m_desired_footpositions_msg);
            RCLCPP_INFO(rclcpp::get_logger("march_gait_planning"), "Home stand position published!");
            break;

        case exoState::BootUp :
            // RCLCPP_INFO(rclcpp::get_logger("march_gait_planning"), "BootUp state entered, waiting for new state."); 
            break;
        
        case exoState::Walk :
            if (m_current_trajectory.empty()) {
                sendRequest(true);
            }
            else{
                std::array<double, 4> current_step = m_current_trajectory.front();
                m_current_trajectory.erase(m_current_trajectory.begin());
                RCLCPP_INFO(rclcpp::get_logger("march_gait_planning"), "Current stance foot is= %d", m_gait_planning.getCurrentStanceFoot());
                if (m_gait_planning.getCurrentStanceFoot() == -1 || m_gait_planning.getCurrentStanceFoot() == 0){ 
                    // -1 is left stance leg, 0 is both
                    setFootPositions(current_step[2], m_gait_planning.getCurrentLeftFootPos()[1], current_step[3], 
                                    current_step[0], m_gait_planning.getCurrentRightFootPos()[1], current_step[1]);
                } else if (m_gait_planning.getCurrentStanceFoot() == 1){
                    // 1 is right stance leg
                    setFootPositions(current_step[0], m_gait_planning.getCurrentLeftFootPos()[1], current_step[1], 
                                    current_step[2], m_gait_planning.getCurrentRightFootPos()[1], current_step[3]);
                }
                m_iks_foot_positions_publisher->publish(*m_desired_footpositions_msg);
                RCLCPP_INFO(rclcpp::get_logger("march_gait_planning"), "Foot positions published!"); 
            }
            break;
    }
}


void GaitPlanningNode::timerCallback() {
    // This code will be executed every 50 milliseconds
    if( m_response_received ){
        // If the response from the server (current stance) has been received, publish the next foot positions.
        footPositionsPublish();
    }
    else{
        // When a request is sent, the response_received is set to false.
        RCLCPP_INFO(rclcpp::get_logger("march_gait_planning"), "Response not received yet, waiting for stance leg."); 
    }
    
}

int main(int argc, char *argv[]){
    
    rclcpp::init(argc, argv); 

    rclcpp::spin(std::make_shared<GaitPlanningNode>()); 
    rclcpp::shutdown(); 

    return 0; 
}

