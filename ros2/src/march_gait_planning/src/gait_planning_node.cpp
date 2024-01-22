/*Authors: Femke Buiks and Andrew Hutani, MIX*/

#include "march_gait_planning/gait_planning_node.hpp"
#include "../../march_mode_machine/include/march_mode_machine/exo_mode.hpp"

using std::placeholders::_1; 

GaitPlanningNode::GaitPlanningNode()
 : Node("march_gait_planning_node"), 
   m_gait_planning(GaitPlanning()),
   m_desired_footpositions_msg(std::make_shared<march_shared_msgs::msg::IksFootPositions>())
 {
    m_iks_foot_positions_publisher = create_publisher<march_shared_msgs::msg::IksFootPositions>("ik_solver/buffer/input", 10);

    m_exo_mode_subscriber = create_subscription<march_shared_msgs::msg::ExoMode>(
        "current_mode", 10, std::bind(&GaitPlanningNode::currentModeCallback, this, _1)); 
    m_exo_joint_state_subscriber = create_subscription<march_shared_msgs::msg::StateEstimation>(
        "state_estimation/state", 100, std::bind(&GaitPlanningNode::currentExoJointStateCallback, this, _1)); 

    m_gait_planning.setGaitType(exoMode::BootUp); 

    m_home_stand = {0.0, 0.16, -0.802, 0.0, -0.16, -0.802}; 

    // auto timer_callback = std::bind(&GaitPlanningNode::timerCallback, this);
    // m_timer = this->create_wall_timer(std::chrono::milliseconds(50), timer_callback);
 }

void GaitPlanningNode::currentModeCallback(const march_shared_msgs::msg::ExoMode::SharedPtr msg){
    RCLCPP_INFO(get_logger(), "Received current mode: %d", msg->mode); 
    m_gait_planning.setGaitType((exoMode)msg->mode);
    footPositionsPublish(); 
}

void GaitPlanningNode::currentExoJointStateCallback(const march_shared_msgs::msg::StateEstimation::SharedPtr msg){
    // RCLCPP_INFO(get_logger(), "Received current foot positions");
    std::array<double, 3> new_left_foot_position = {msg->foot_pose[0].position.x, msg->foot_pose[0].position.y, msg->foot_pose[0].position.z};
    std::array<double, 3> new_right_foot_position = {msg->foot_pose[1].position.x, msg->foot_pose[1].position.y, msg->foot_pose[1].position.z};
    m_gait_planning.setFootPositions(new_left_foot_position, new_right_foot_position); 
    if (m_current_trajectory.empty()){
        m_gait_planning.setStanceFoot(msg->stance_leg); 
        // RCLCPP_INFO(get_logger(), "Received current stance foot"); 
    }
    footPositionsPublish(); 
}

void GaitPlanningNode::setFootPositionsMessage(double left_x, double left_y, double left_z, 
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
        case exoMode::Stand :
            m_current_trajectory.clear();
            setFootPositionsMessage(m_home_stand[0], m_home_stand[1], m_home_stand[2], m_home_stand[3], m_home_stand[4], m_home_stand[5]);
            m_iks_foot_positions_publisher->publish(*m_desired_footpositions_msg);
            RCLCPP_INFO(rclcpp::get_logger("march_gait_planning"), "Home stand position published!");
            break;

        case exoMode::BootUp :
            // RCLCPP_INFO(rclcpp::get_logger("march_gait_planning"), "BootUp mode entered, waiting for new mode."); 
            break;
        
        case exoMode::LargeWalk :
        case exoMode::SmallWalk :
            if (m_current_trajectory.empty()) {
                m_current_trajectory = m_gait_planning.getTrajectory(); 
                RCLCPP_INFO(rclcpp::get_logger("march_gait_planning"), "Trajectory refilled!");
            }
            else {
                std::array<double, 4> current_step = m_current_trajectory.front();
                m_current_trajectory.erase(m_current_trajectory.begin());
                RCLCPP_INFO(rclcpp::get_logger("march_gait_planning"), "Current stance foot is= %d", m_gait_planning.getCurrentStanceFoot());
                // if (m_gait_planning.getCurrentStanceFoot() == -1 || m_gait_planning.getCurrentStanceFoot() == 0){
                if (m_gait_planning.getCurrentStanceFoot() & 0b1){
                    // 01 is left stance leg, 11 is both, 00 is neither and 10 is right. 1 as last int means left or both. 
                    setFootPositionsMessage(current_step[2], m_home_stand[1], current_step[3] + m_home_stand[2], 
                                    current_step[0], m_home_stand[4], current_step[1] + m_home_stand[5]);
                } else if (m_gait_planning.getCurrentStanceFoot() & 0b10){
                    // 10 is right stance leg
                    setFootPositionsMessage(current_step[0], m_home_stand[1], current_step[1]+ m_home_stand[2], 
                                    current_step[2], m_home_stand[4], current_step[3]+ m_home_stand[5]);
                }
                m_iks_foot_positions_publisher->publish(*m_desired_footpositions_msg);
                RCLCPP_INFO(rclcpp::get_logger("march_gait_planning"), "Foot positions published!"); 
            }
            break;
    }
}

// void GaitPlanningNode::timerCallback() {
//     footPositionsPublish();
// }

int main(int argc, char *argv[]){
    
    rclcpp::init(argc, argv); 

    rclcpp::spin(std::make_shared<GaitPlanningNode>()); 
    rclcpp::shutdown(); 

    return 0; 
}

