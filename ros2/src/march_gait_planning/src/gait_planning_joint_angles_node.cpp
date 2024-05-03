/*
Authors: Femke Buiks, MIX

This file contains the functionality of the GaitPlanningAnglesNode. This node serves to supply gaits to the exo that are predetermined joint angles (from csv files).
Gait logic is mainly located here, as the publishing of gaits is dependent on callbacks from the state machine. 

*/ 

#include "march_gait_planning/gait_planning_joint_angles_node.hpp"

using std::placeholders::_1; 

int INTERPOLATING_TIMESTEPS = 40;

struct CSVRow {
    std::string left_hip_aa;
    std::string left_hip_fe;
    std::string left_knee; 
    std::string left_ankle; 
    std::string right_hip_aa;
    std::string right_hip_fe;
    std::string right_knee; 
    std::string right_ankle;
};

GaitPlanningAnglesNode::GaitPlanningAnglesNode()
 : Node("gait_planning_angles_node"), 
   m_gait_planning(GaitPlanningAngles()),
   m_joints_msg(),
   m_current_trajectory(),
   m_incremental_steps_to_home_stand(),
   m_first_stand(true),
   m_initial_point()
{
    m_current_state_subscriber = create_subscription<march_shared_msgs::msg::StateEstimation>("state_estimation/state", 10, std::bind(&GaitPlanningAnglesNode::currentJointAnglesCallback, this, _1)); 
    
    m_exo_mode_subscriber = create_subscription<march_shared_msgs::msg::ExoMode>("current_mode", 10, std::bind(&GaitPlanningAnglesNode::currentModeCallback, this, _1)); 
    // m_joint_angle_trajectory_publisher = create_publisher<trajectory_msgs::msg::JointTrajectory>("joint_trajectory_controller/joint_trajectory", 10); 
    m_joint_angle_trajectory_publisher = create_publisher<std_msgs::msg::Float64MultiArray>("march_joint_position_controller/commands", 10); 
    RCLCPP_INFO(rclcpp::get_logger("march_gait_planning"), "Joint trajectory publisher created"); 
    
    // auto timer_publish = std::bind(&GaitPlanningAnglesNode::publishJointTrajectoryPoints, this);
    // m_timer = this->create_wall_timer(std::chrono::milliseconds(50), timer_publish);
    // std::cout << "Timer function created" << std::endl; 

    m_gait_planning.setGaitType(exoMode::BootUp);
    m_gait_planning.setPrevGaitType(exoMode::BootUp); 
    m_gait_planning.setHomeStand(m_gait_planning.getFirstStepAngleTrajectory()[0]); 

    RCLCPP_INFO(rclcpp::get_logger("march_gait_planning"), "Gait planning node initialized");
}



void GaitPlanningAnglesNode::currentModeCallback(const march_shared_msgs::msg::ExoMode::SharedPtr msg){
    RCLCPP_INFO(rclcpp::get_logger("march_gait_planning"), "Received current mode: %s", toString(static_cast<exoMode>(msg->mode)).c_str()); 
    m_gait_planning.setPrevGaitType(m_gait_planning.getGaitType());
    m_gait_planning.setGaitType((exoMode)msg->mode);
    // DO NOT set counter to 0 if you switch from walking to standing (prev type is walk and new type is stand) 
    if ((exoMode)msg->mode == exoMode::Stand){
        if (m_gait_planning.getPrevGaitType() == exoMode::Walk || m_gait_planning.getPrevGaitType() == exoMode::Ascending || m_gait_planning.getPrevGaitType() == exoMode::Descending || m_gait_planning.getPrevGaitType() == exoMode::Sideways){
        }   
    } else {
        m_gait_planning.setCounter(0); 
        RCLCPP_DEBUG(this->get_logger(), "setting counter to 0 in this gait switch!"); 
    }
    if (!m_first_stand){
        publishJointTrajectoryPoints(); 
    }
    // publishJointTrajectoryPoints(); 
}

void GaitPlanningAnglesNode::currentJointAnglesCallback(const march_shared_msgs::msg::StateEstimation::SharedPtr msg) {
    if (m_first_stand && m_gait_planning.getGaitType() == exoMode::Stand) {
        std::vector<double> point = msg->joint_state.position; 
        if (point.size() >= 8) {
            // TODO: overhaul remapping once an order is chosen!
            m_gait_planning.setPrevPoint({point[1], point[2], point[3], point[0], point[5], point[6], point[7], point[4]}); 
            // m_gait_planning.setPrevPoint(point); 
            RCLCPP_INFO(rclcpp::get_logger("march_gait_planning"), "Received current joint angles"); 
            std::string point_str;
            for (const auto &value : m_gait_planning.getPrevPoint()) {
                point_str += std::to_string(value) + " ";
            }

            RCLCPP_INFO(rclcpp::get_logger("march_gait_planning"), "Point values: %s", point_str.c_str());
            m_first_stand = false;
        } else {
            RCLCPP_INFO(rclcpp::get_logger("march_gait_planning"), "Not enough joint angles to set previous point correctly!");
        }
    }
    publishJointTrajectoryPoints(); 
    
}

void GaitPlanningAnglesNode::processMovingGaits(const int &counter){
    if (!m_current_trajectory.empty()){
        m_joints_msg.data = remapKinematicChaintoAlphabetical(m_current_trajectory[counter]);
        m_joint_angle_trajectory_publisher->publish(m_joints_msg);
        RCLCPP_DEBUG(rclcpp::get_logger("march_gait_planning"), "Gait message published!"); 
    }
} 

std::vector<double> GaitPlanningAnglesNode::remapKinematicChaintoAlphabetical(const std::vector<double> &kinematic_vector){
    return {kinematic_vector[3], kinematic_vector[0], kinematic_vector[1], kinematic_vector[2],
            kinematic_vector[7], kinematic_vector[4], kinematic_vector[5], kinematic_vector[6]};
}

void GaitPlanningAnglesNode::processHomeStandGait(){
    if (m_gait_planning.getCounter() == 0){ // When switching to homestand
        m_incremental_steps_to_home_stand.clear();
        for (unsigned i = 0; i < m_gait_planning.getHomeStand().size(); ++i) {
                m_incremental_steps_to_home_stand.push_back((m_gait_planning.getHomeStand()[i] - m_gait_planning.getPrevPoint()[i]) / INTERPOLATING_TIMESTEPS); // 40 iterations to reach the target, i.e. in 2 seconds
        }
        m_initial_point = m_gait_planning.getPrevPoint();
        RCLCPP_DEBUG(rclcpp::get_logger("march_gait_planning"), "Increments correctly calculated!");
    }
    std::vector<double> temp_moving_to_home_stand;
   
    if (m_gait_planning.getCounter() < INTERPOLATING_TIMESTEPS){
        RCLCPP_DEBUG(rclcpp::get_logger("march_gait_planning"), "Moving towards home stand!");
        for (unsigned i = 0; i < m_gait_planning.getHomeStand().size(); ++i) {
            m_initial_point[i] += m_incremental_steps_to_home_stand[i];
            temp_moving_to_home_stand.push_back(m_initial_point[i]);
        } 

    }
    else{
        RCLCPP_DEBUG(rclcpp::get_logger("march_gait_planning"), "Home stand position reached!");
        temp_moving_to_home_stand = m_gait_planning.getHomeStand();
        m_initial_point.clear();
    }

    m_joints_msg.data = remapKinematicChaintoAlphabetical(temp_moving_to_home_stand);
    m_joint_angle_trajectory_publisher->publish(m_joints_msg);
    
    m_gait_planning.setCounter(m_gait_planning.getCounter() + 1);


}

void GaitPlanningAnglesNode::finishGaitBeforeStand(){
    unsigned count = m_gait_planning.getCounter(); 
    if (count < m_current_trajectory.size()-1){ 
        processMovingGaits(count); 
        m_gait_planning.setCounter(count+1); 
        RCLCPP_DEBUG(this->get_logger(), "Finishing gait, with count: %d", count);
    } if (count == m_current_trajectory.size()-1) { 
        m_joints_msg.data = remapKinematicChaintoAlphabetical(m_gait_planning.getHomeStand()); 
        m_joint_angle_trajectory_publisher->publish(m_joints_msg);
        m_gait_planning.setPrevPoint(m_gait_planning.getHomeStand());
        RCLCPP_DEBUG(this->get_logger(), "Publishing home stand");
    } 
}


void GaitPlanningAnglesNode::publishJointTrajectoryPoints(){
    if (!m_first_stand){
        RCLCPP_DEBUG(this->get_logger(), "Entering timer function"); 
        unsigned count = m_gait_planning.getCounter(); 
        switch (m_gait_planning.getGaitType()) {
            case exoMode::Walk : 
            
            switch(m_gait_planning.getPrevGaitType()){
                case exoMode::Walk : 

                    m_current_trajectory = m_gait_planning.getFullGaitAngleCSV(); 
                    processMovingGaits(count);
                    if (count >= (m_current_trajectory.size()-1)){
                        m_gait_planning.setCounter(0);
                    } else {
                        m_gait_planning.setCounter(count+1);
                    }
                    break; 

                case exoMode::Stand :

                    m_current_trajectory = m_gait_planning.getFirstStepAngleTrajectory(); 
                    processMovingGaits(count);
                    if (count >= (m_current_trajectory.size()-1)){
                        m_gait_planning.setCounter(0); 
                        m_gait_planning.setPrevGaitType(exoMode::Walk); 
                    } else {
                        m_gait_planning.setCounter(count+1);
                    }
                    break; 

                default :

                    break; 
                }

                break; 
            
            case exoMode::Stand :

                switch(m_gait_planning.getPrevGaitType()){

                    case exoMode::Sit :

                        m_current_trajectory = m_gait_planning.getSitToStandGait(); 
                        processMovingGaits(count);
                        if (count >= (m_current_trajectory.size()-1)){
                            m_gait_planning.setCounter(m_current_trajectory.size()-1);
                            m_gait_planning.setPrevGaitType(exoMode::Stand); 
                        } else {
                            m_gait_planning.setCounter(count+1);
                        }
                        break; 

                    case exoMode::Hinge :
                        
                        m_current_trajectory = m_gait_planning.getHingeGait(); 
                        processMovingGaits(count); 
                        RCLCPP_INFO(rclcpp::get_logger("march_gait_planning"), "Count: %d", count);
                        if (count <= 0){
                            m_gait_planning.setCounter(0); 
                            m_current_trajectory.clear();
                            m_gait_planning.setPrevGaitType(exoMode::Stand); 
                        } else {
                            m_gait_planning.setCounter(count-1);
                        }
                        break;

                    case exoMode::Walk :

                        if (count < m_current_trajectory.size()-1){
                            RCLCPP_DEBUG(this->get_logger(), "finishing gait! with count %d", count); 
                            processMovingGaits(count); 
                            m_gait_planning.setCounter(count+1); 
                        } if (count >= m_current_trajectory.size()-1){
                            m_gait_planning.setCounter(0); 
                            m_current_trajectory = m_gait_planning.getStepCloseGait();
                            RCLCPP_DEBUG(this->get_logger(), "Filled step close trajectory"); 
                            m_gait_planning.setPrevGaitType(exoMode::Stand); 
                        }
                        break; 

                    case exoMode::Stand :

                        finishGaitBeforeStand(); 
                        break; 

                    case exoMode::BootUp :

                        processHomeStandGait(); 
                        break; 

                    case exoMode::Ascending :

                        finishGaitBeforeStand(); 
                        break; 

                    case exoMode::Descending :

                        finishGaitBeforeStand(); 
                        break; 

                    case exoMode::Sideways :

                        finishGaitBeforeStand(); 
                        break; 

                    default :

                        break; 
                }
                break; 
            
            case exoMode::Sit :
                m_current_trajectory = m_gait_planning.getStandToSitGait(); 
                processMovingGaits(count);
                if (count >= (m_current_trajectory.size()-1)){
                    m_gait_planning.setCounter(m_current_trajectory.size()-1); 
                } else {
                    m_gait_planning.setCounter(count+1);
                }
                break;

            case exoMode::Hinge:
                m_current_trajectory = m_gait_planning.getHingeGait(); 
                processMovingGaits(count); 
                if (count >= (m_current_trajectory.size()-1)){
                    m_gait_planning.setCounter(m_current_trajectory.size()-1); 
                } else {
                    m_gait_planning.setCounter(count+1);
                }
                break;
            
            case exoMode::Sideways :
                m_current_trajectory = m_gait_planning.getSidewaysGait(); 
                processMovingGaits(count); 
                if (count >= (m_current_trajectory.size()-1)){
                    m_gait_planning.setCounter(0); 
                } else {
                    m_gait_planning.setCounter(count+1); 
                }
                break;

            case exoMode::Ascending :
                m_current_trajectory = m_gait_planning.getAscendingGait(); 
                if (count < m_current_trajectory.size()-1){
                    processMovingGaits(count); 
                    m_gait_planning.setCounter(count+1); 
                }
                break;

            case exoMode::Descending :
                m_current_trajectory = m_gait_planning.getDescendingGait(); 
                if (count < m_current_trajectory.size()-1){
                    processMovingGaits(count); 
                    m_gait_planning.setCounter(count+1); 
                }
                break;

            default :
                // RCLCPP_INFO(rclcpp::get_logger("march_gait_planning"), "Waiting for command"); 
                break;
        }
    }
}

int main(int argc, char *argv[]){
    
    rclcpp::init(argc, argv); 

    rclcpp::spin(std::make_shared<GaitPlanningAnglesNode>()); 
    rclcpp::shutdown(); 

    return 0; 
}
