/*
Authors: Femke Buiks, MIX

This file contains the functionality of the GaitPlanningAnglesNode. This node serves to supply gaits to the exo that are predetermined joint angles (from csv files).
Gait logic is mainly located here, as the publishing of gaits is dependent on callbacks from the state machine. 

*/ 

#include "march_gait_planning/gait_planning_joint_angles_node.hpp"

using std::placeholders::_1; 

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
   m_trajectory_prev_point(),
   m_trajectory_des_point(),
   m_current_trajectory(),
   m_incremental_steps_to_home_stand(),
   m_first_stand(true)
{
    m_joints_msg.joint_names = {"left_hip_aa", "left_hip_fe", "left_knee", "left_ankle", 
                        "right_hip_aa", "right_hip_fe", "right_knee", "right_ankle"};
    initializeConstantsPoints(m_trajectory_prev_point);
    initializeConstantsPoints(m_trajectory_des_point); 
    m_trajectory_des_point.time_from_start.nanosec = int(50*1e6); 

    m_exo_state_subscriber = create_subscription<march_shared_msgs::msg::ExoState>("current_state", 10, std::bind(&GaitPlanningAnglesNode::currentStateCallback, this, _1)); 
    m_joint_angle_trajectory_publisher = create_publisher<trajectory_msgs::msg::JointTrajectory>("joint_trajectory_controller/joint_trajectory", 10); 
    std::cout << "Joint trajectory publisher created" << std::endl; 
    
    m_get_current_joint_angles_client = 
        create_client<march_shared_msgs::srv::GetCurrentJointPositions>("get_previous_joint_positions");
    while (!m_get_current_joint_angles_client->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_WARN(this->get_logger(), "Waiting for service get_previous_joint_positions to become available...");
    }
    RCLCPP_INFO(this->get_logger(), "Connected to service get_previous_joint_positions");

    auto timer_publish = std::bind(&GaitPlanningAnglesNode::publishJointTrajectoryPoints, this);
    m_timer = this->create_wall_timer(std::chrono::milliseconds(50), timer_publish);
    std::cout << "Timer function created" << std::endl; 

    m_gait_planning.setGaitType(exoState::BootUp);
    m_gait_planning.setPrevGaitType(exoState::BootUp); 
    m_gait_planning.setHomeStand(m_gait_planning.getFirstStepAngleTrajectory()[0]); 
}

std::vector<double> GaitPlanningAnglesNode::getCurrentJointAngles(){
    auto request = std::make_shared<march_shared_msgs::srv::GetCurrentJointPositions::Request>();
    auto result_future = m_get_current_joint_angles_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(get_node_base_interface(), result_future) ==
        rclcpp::executor::FutureReturnCode::SUCCESS)
    {
        auto result = result_future.get();
        return result->joint_positions;
    }
    else {
        RCLCPP_ERROR(this->get_logger(), "Failed to call service GetCurrentJointAngles");
        return {};
    }
}

void GaitPlanningAnglesNode::currentStateCallback(const march_shared_msgs::msg::ExoState::SharedPtr msg){
    RCLCPP_INFO(rclcpp::get_logger("march_gait_planning"), "received current state: %d", msg->state); 
    m_gait_planning.setPrevGaitType(m_gait_planning.getGaitType());
    m_gait_planning.setGaitType((exoState)msg->state); 
    m_gait_planning.setCounter(0); 
    publishJointTrajectoryPoints(); 
}

void GaitPlanningAnglesNode::initializeConstantsPoints(trajectory_msgs::msg::JointTrajectoryPoint &point){
    point.velocities = {0, 0, 0, 0, 0, 0, 0, 0}; 
    point.accelerations = {0, 0, 0, 0, 0, 0, 0, 0}; 
    point.effort = {0, 0, 0, 0, 0, 0, 0, 0}; 
    point.time_from_start.sec = 0; 
    point.time_from_start.nanosec = 0;
}

void GaitPlanningAnglesNode::processMovingGaits(const int &counter){
    if (!m_current_trajectory.empty()){
        m_trajectory_prev_point.positions = m_gait_planning.getPrevPoint(); 
        m_joints_msg.points.push_back(m_trajectory_prev_point); 
        m_trajectory_des_point.positions = m_current_trajectory[counter]; 
        m_joints_msg.points.push_back(m_trajectory_des_point);
        m_gait_planning.setPrevPoint(m_current_trajectory[counter]); 
        m_joint_angle_trajectory_publisher->publish(m_joints_msg);
        RCLCPP_INFO(rclcpp::get_logger("march_gait_planning"), "Gait message published!"); 
        m_joints_msg.points.clear();   
    }
} 

void GaitPlanningAnglesNode::processHomeStandGait(){
    if (m_gait_planning.getCounter() == 0){ // When switching to homestand
        m_incremental_steps_to_home_stand.clear();
        for (int i = 0; i < m_gait_planning.getHomeStand().size(); ++i) {
                m_incremental_steps_to_home_stand.push_back((m_gait_planning.getHomeStand()[i] - m_gait_planning.getPrevPoint()[i]) / 40); // 40 iterations to reach the target, i.e. in 2 seconds
        }
    }

    m_trajectory_prev_point.positions = m_gait_planning.getPrevPoint(); 
    m_joints_msg.points.push_back(m_trajectory_prev_point); 

    if (m_gait_planning.getCounter() < 40){
        m_trajectory_des_point.positions.clear();
        RCLCPP_INFO(rclcpp::get_logger("march_gait_planning"), "Moving towards home stand!");
        for (int i = 0; i < m_gait_planning.getHomeStand().size(); ++i) {
            m_trajectory_des_point.positions.push_back(m_gait_planning.getPrevPoint()[i] + m_incremental_steps_to_home_stand[i]);
        } 
    }
    else{
        RCLCPP_INFO(rclcpp::get_logger("march_gait_planning"), "Home stand position reached!");
        m_trajectory_des_point.positions = m_gait_planning.getHomeStand();
    }

    m_joints_msg.points.push_back(m_trajectory_des_point);
    m_joint_angle_trajectory_publisher->publish(m_joints_msg);
    
    m_joints_msg.points.clear();   
    m_gait_planning.setPrevPoint(m_trajectory_des_point.positions); 
    m_gait_planning.setCounter(m_gait_planning.getCounter() + 1);

}


void GaitPlanningAnglesNode::publishJointTrajectoryPoints(){
    int count = m_gait_planning.getCounter(); 
    switch (m_gait_planning.getGaitType()) {
        case exoState::Walk : 
            if (m_gait_planning.getPrevGaitType() == exoState::Walk){
                m_current_trajectory = m_gait_planning.getFullGaitAngleCSV(); 
                processMovingGaits(count);
                if (count >= (m_current_trajectory.size()-1)){
                    m_gait_planning.setCounter(0);
                } else {
                    m_gait_planning.setCounter(count+1);
                } 
            } else if (m_gait_planning.getPrevGaitType() == exoState::Stand) {
                m_current_trajectory = m_gait_planning.getFirstStepAngleTrajectory(); 
                processMovingGaits(count);
                if (count >= (m_current_trajectory.size()-1)){
                    m_gait_planning.setCounter(0); 
                    m_gait_planning.setPrevGaitType(exoState::Walk); 
                } else {
                    m_gait_planning.setCounter(count+1);
                }
            }
            break;
        
        case exoState::Stand :
            if (m_gait_planning.getPrevGaitType() == exoState::Sit){
                m_current_trajectory = m_gait_planning.getSitToStandGait(); 
                processMovingGaits(count);
                if (count >= (m_current_trajectory.size()-1)){
                    m_gait_planning.setCounter(m_current_trajectory.size()-1);
                    m_gait_planning.setPrevGaitType(exoState::Stand); 
                } else {
                    m_gait_planning.setCounter(count+1);
                } 
            } else {
                if (m_first_stand){
                    std::vector<double> current_joint_positions = getCurrentJointAngles();
                    m_gait_planning.setPrevPoint(current_joint_positions);
                    m_first_stand = false;
                    RCLCPP_ERROR(this->get_logger(), "Previous point set to current joint angles");
                }
                processHomeStandGait(); 
            }
            break; 
        
        case exoState::Sit :
            m_current_trajectory = m_gait_planning.getStandToSitGait(); 
            processMovingGaits(count);
            if (count >= (m_current_trajectory.size()-1)){
                m_gait_planning.setCounter(m_current_trajectory.size()-1); 
            } else {
                m_gait_planning.setCounter(count+1);
            }
            break;
        
        case exoState::Sideways :
            m_current_trajectory = m_gait_planning.getSidewaysGait(); 
            processMovingGaits(count); 
            if (count >= (m_current_trajectory.size()-1)){
                m_gait_planning.setCounter(0); 
            } else {
                m_gait_planning.setCounter(count+1); 
            }
        
        default :
            // RCLCPP_INFO(rclcpp::get_logger("march_gait_planning"), "Waiting for command"); 
            break;
    }
}

int main(int argc, char *argv[]){
    
    rclcpp::init(argc, argv); 

    rclcpp::spin(std::make_shared<GaitPlanningAnglesNode>()); 
    rclcpp::shutdown(); 

    return 0; 
}
