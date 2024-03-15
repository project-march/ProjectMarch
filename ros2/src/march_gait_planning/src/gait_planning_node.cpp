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

    // m_variable_foot_step_subscriber = create_subscription<march_shared_msgs::msg::FootStepOutput>("footsteps", 100, std::bind(&GaitPlanningNode::variableFootstepCallback, this, _1)); 

    m_mpc_foot_positions_subscriber = create_subscription<geometry_msgs::msg::PoseArray>("final_feet_position", 10, std::bind(&GaitPlanningNode::variableFootstepCallback, this, _1)); 

    m_gait_planning.setGaitType(exoMode::BootUp); 

    m_home_stand = {0.3, 0.12, -0.65, 0.3, -0.12, -0.65}; 
 }

void GaitPlanningNode::currentModeCallback(const march_shared_msgs::msg::ExoMode::SharedPtr msg){
    RCLCPP_INFO(get_logger(), "Received current mode: %s", toString(static_cast<exoMode>(msg->mode)).c_str()); 
    m_gait_planning.setPreviousGaitType(m_gait_planning.getGaitType()); 
    m_gait_planning.setGaitType((exoMode)msg->mode);
    footPositionsPublish(); 
}

void GaitPlanningNode::currentExoJointStateCallback(const march_shared_msgs::msg::StateEstimation::SharedPtr msg){
    // RCLCPP_INFO(get_logger(), "Received current foot positions");
    GaitPlanningNode::XYZFootPositionArray new_left_foot_position = {msg->foot_pose[0].position.x, msg->foot_pose[0].position.y, msg->foot_pose[0].position.z};
    GaitPlanningNode::XYZFootPositionArray new_right_foot_position = {msg->foot_pose[1].position.x, msg->foot_pose[1].position.y, msg->foot_pose[1].position.z};
    m_gait_planning.setFootPositions(new_left_foot_position, new_right_foot_position); 
    m_desired_footpositions_msg->header = msg->header;
    if (m_current_trajectory.empty()){
        m_gait_planning.setStanceFoot(msg->stance_leg); 
        // RCLCPP_INFO(rclcpp::get_logger("march_gait_planning"), "Current stance foot is= %d", m_gait_planning.getCurrentStanceFoot());
    }
    footPositionsPublish(); 
}

void GaitPlanningNode::variableFootstepCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg){
    //Remove duplicates from PoseArray message to identify the two desired footsteps
    std::set<geometry_msgs::msg::Pose, GaitPlanningNode::PoseXComparator> final_feet(msg->poses.begin(), msg->poses.end());

    geometry_msgs::msg::Pose foot_pos = *final_feet.begin(); 
    // determine if left or right, maybe by tracking the previous desired step? 
    float dist = foot_pos.position.x - m_gait_planning.getCurrentRightFootPos()[0]; 
    m_current_trajectory.clear(); 
    m_current_trajectory = m_gait_planning.interpolateVariableTrajectory(dist); 
    footPositionsPublish(); 

    // When distance coming from mode machine 
    // float dist = msg->distance; 
    // m_current_trajectory.clear(); 
    // m_current_trajectory = m_gait_planning.interpolateVariableTrajectory(dist); 
    // footPositionsPublish(); 
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

//TODO: add Step Close function. Upon calling this function, also get the feet positions once upon starting the step close
// and add these coordinates to the step close vector, to make sure the trajectory is defined w/ respect to body frame. 
void GaitPlanningNode::footPositionsPublish(){
    switch (m_gait_planning.getGaitType()){
        case exoMode::Stand :
            // If the current trajectory is not empty, finish it before going to step close or stand.
            if (!m_current_trajectory.empty()){
                RCLCPP_DEBUG_THROTTLE(rclcpp::get_logger("march_gait_planning"), *this->get_clock(), 100, "Finishing current trajectory before standing."); 
                GaitPlanningNode::XZFeetPositionsArray current_step = m_current_trajectory.front();
                m_current_trajectory.erase(m_current_trajectory.begin());
                if (m_gait_planning.getCurrentStanceFoot() & 0b1){
                    // 01 is left stance leg, 11 is both, 00 is neither and 10 is right. 1 as last int means left or both. 
                    setFootPositionsMessage(current_step[2]+m_home_stand[0], m_home_stand[1], current_step[3]+m_home_stand[2], 
                                    current_step[0]+m_home_stand[3], m_home_stand[4], current_step[1]+m_home_stand[5]);
                } else if (m_gait_planning.getCurrentStanceFoot() & 0b10){
                    // 10 is right stance leg
                    setFootPositionsMessage(current_step[0]+m_home_stand[0], m_home_stand[1], current_step[1]+m_home_stand[2], 
                                    current_step[2]+m_home_stand[3], m_home_stand[4], current_step[3]+m_home_stand[5]);
                }
                m_iks_foot_positions_publisher->publish(*m_desired_footpositions_msg);
            }

            // Step close if previous gait was walk. Note how variable walk already has a step close implemented by default.
            else if (m_gait_planning.getPreviousGaitType() == exoMode::LargeWalk || m_gait_planning.getPreviousGaitType() == exoMode::SmallWalk){
                RCLCPP_DEBUG(rclcpp::get_logger("march_gait_planning"), "Calling step close trajectory with mode: %d", m_gait_planning.getPreviousGaitType());
                m_current_trajectory = m_gait_planning.getTrajectory();
                RCLCPP_DEBUG(rclcpp::get_logger("march_gait_planning"), "Size of step close trajectory: %d", m_current_trajectory.size());
                m_gait_planning.setPreviousGaitType(exoMode::Stand);
            }

            else {
                m_current_trajectory.clear();
                setFootPositionsMessage(m_home_stand[0], m_home_stand[1], m_home_stand[2], m_home_stand[3], m_home_stand[4], m_home_stand[5]);
                m_iks_foot_positions_publisher->publish(*m_desired_footpositions_msg);
                RCLCPP_INFO_THROTTLE(rclcpp::get_logger("march_gait_planning"), *this->get_clock(), 2000, "Publishing homestand position.");
            }
            break;

        case exoMode::BootUp :
            RCLCPP_DEBUG(rclcpp::get_logger("march_gait_planning"), "BootUp mode entered, waiting for new mode."); 
            break;
        
        case exoMode::LargeWalk :
        case exoMode::SmallWalk :
            if (m_current_trajectory.empty()) {
                m_current_trajectory = m_gait_planning.getTrajectory(); 
                RCLCPP_INFO(rclcpp::get_logger("march_gait_planning"), "Trajectory refilled!");
            }
            else {
                GaitPlanningNode::XZFeetPositionsArray current_step = m_current_trajectory.front();
                m_current_trajectory.erase(m_current_trajectory.begin());
                if (m_gait_planning.getCurrentStanceFoot() & 0b1){
                    // 01 is left stance leg, 11 is both, 00 is neither and 10 is right. 1 as last int means left or both. 
                    setFootPositionsMessage(current_step[2]+m_home_stand[0], m_home_stand[1], current_step[3]+m_home_stand[2], 
                                    current_step[0]+m_home_stand[3], m_home_stand[4], current_step[1]+m_home_stand[5]);
                } else if (m_gait_planning.getCurrentStanceFoot() & 0b10){
                    // 10 is right stance leg
                    setFootPositionsMessage(current_step[0]+m_home_stand[0], m_home_stand[1], current_step[1]+m_home_stand[2], 
                                    current_step[2]+m_home_stand[3], m_home_stand[4], current_step[3]+m_home_stand[5]);
                }
                m_iks_foot_positions_publisher->publish(*m_desired_footpositions_msg);
            }
            break;

        case exoMode::VariableStep : 
            if (m_current_trajectory.empty()){
               // eventually this will be the stepclose function
                setFootPositionsMessage(m_home_stand[0], m_home_stand[1], m_home_stand[2], m_home_stand[3], m_home_stand[4], m_home_stand[5]);
                m_iks_foot_positions_publisher->publish(*m_desired_footpositions_msg);
            }
            else { 
                GaitPlanningNode::XZFeetPositionsArray current_step = m_current_trajectory.front();
                m_current_trajectory.erase(m_current_trajectory.begin());
                setFootPositionsMessage(current_step[2]+m_home_stand[0], m_home_stand[1], current_step[3] + m_home_stand[2], 
                                current_step[0]+m_home_stand[3], m_home_stand[4], current_step[1] + m_home_stand[5]);
                m_iks_foot_positions_publisher->publish(*m_desired_footpositions_msg);
            } 
            break;
        
        case exoMode::VariableWalk :
        //input from MPC comes in here. 

        default :
            break; 
    }
}

int main(int argc, char *argv[]){
    
    rclcpp::init(argc, argv); 

    rclcpp::spin(std::make_shared<GaitPlanningNode>()); 
    rclcpp::shutdown(); 

    return 0; 
}

