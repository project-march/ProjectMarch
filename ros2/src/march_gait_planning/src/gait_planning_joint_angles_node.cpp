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
   m_gait_planning(GaitPlanningAngles())
{
    m_exo_state_subscriber = create_subscription<march_shared_msgs::msg::ExoState>("current_state", 10, std::bind(&GaitPlanningAnglesNode::currentStateCallback, this, _1)); 
    m_joint_angle_trajectory_publisher = create_publisher<trajectory_msgs::msg::JointTrajectory>("joint_trajectory_controller/joint_trajectory", 10); 
    std::cout << "Joint trajectory publisher created" << std::endl; 
    
    auto timer_publish = std::bind(&GaitPlanningAnglesNode::publishJointTrajectoryPoints, this);
    m_timer = this->create_wall_timer(std::chrono::milliseconds(50), timer_publish);
    std::cout << "Timer function created" << std::endl; 

    m_gait_planning.setGaitType(exoState::BootUp);
    m_gait_planning.setPrevGaitType(exoState::BootUp); 
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

void GaitPlanningAnglesNode::processContinuousWalkingGait(trajectory_msgs::msg::JointTrajectoryPoint &prev_point, trajectory_msgs::msg::JointTrajectoryPoint &des_point, trajectory_msgs::msg::JointTrajectory &message){
    if (!m_gait_planning.getFullGaitAngleCSV().empty()){

        std::vector<std::vector<double>> trajectory = m_gait_planning.getFullGaitAngleCSV();
        int count = m_gait_planning.getCounter();  

        prev_point.positions = m_gait_planning.getPrevPoint(); 
        message.points.push_back(prev_point); 

        des_point.positions = trajectory[count]; 
        message.points.push_back(des_point);

        m_gait_planning.setPrevPoint(trajectory[count]); 

        m_joint_angle_trajectory_publisher->publish(message);
        RCLCPP_INFO(rclcpp::get_logger("march_gait_planning"), "Complete gait message published!"); 

        if (count >= (m_gait_planning.getFullGaitAngleCSV().size()-1)){
            m_gait_planning.setCounter(0);
        } else {
            m_gait_planning.setCounter(count+1);
        }
    }
}

void GaitPlanningAnglesNode::processFirstStepWalkingGait(trajectory_msgs::msg::JointTrajectoryPoint &prev_point, trajectory_msgs::msg::JointTrajectoryPoint &des_point, trajectory_msgs::msg::JointTrajectory &message){
    if (!m_gait_planning.getFirstStepAngleTrajectory().empty()){

        trajectory_msgs::msg::JointTrajectory msg;

        std::vector<std::vector<double>> trajectory = m_gait_planning.getFirstStepAngleTrajectory();
        int count = m_gait_planning.getCounter();  

        prev_point.positions = m_gait_planning.getPrevPoint(); 
        message.points.push_back(prev_point); 

        des_point.positions = trajectory[count]; 
        message.points.push_back(des_point);

        m_gait_planning.setPrevPoint(trajectory[count]); 

        m_joint_angle_trajectory_publisher->publish(message);
        RCLCPP_INFO(rclcpp::get_logger("march_gait_planning"), "First step message published!"); 

        if (count >= (m_gait_planning.getFirstStepAngleTrajectory().size()-1)){
            m_gait_planning.setCounter(0); 
            m_gait_planning.setPrevGaitType(exoState::Walk); 
        } else {
            m_gait_planning.setCounter(count+1);
        }
    }
}

void GaitPlanningAnglesNode::processHomeStandGait(trajectory_msgs::msg::JointTrajectoryPoint &prev_point, trajectory_msgs::msg::JointTrajectoryPoint &des_point, trajectory_msgs::msg::JointTrajectory &message){
    m_gait_planning.setPrevPoint({0.006, 0.042, -0.0, -0.016, -0.006, 0.042, -0.0, -0.016});

    prev_point.positions = m_gait_planning.getPrevPoint();
    message.points.push_back(prev_point);

    des_point.positions = {0.006, 0.042, -0.0, -0.016, -0.006, 0.042, -0.0, -0.016};
    message.points.push_back(des_point);

    m_joint_angle_trajectory_publisher->publish(message);
    RCLCPP_INFO(rclcpp::get_logger("march_gait_planning"), "Home stand message published!");
    m_gait_planning.setCounter(0);
}

void GaitPlanningAnglesNode::processStandToSitGait(trajectory_msgs::msg::JointTrajectoryPoint &prev_point, trajectory_msgs::msg::JointTrajectoryPoint &des_point, trajectory_msgs::msg::JointTrajectory &message){
    if (!m_gait_planning.getStandToSitGait().empty()){

        std::vector<std::vector<double>> trajectory = m_gait_planning.getStandToSitGait();
        int count = m_gait_planning.getCounter();  

        prev_point.positions = m_gait_planning.getPrevPoint();  
        message.points.push_back(prev_point); 

        des_point.positions = trajectory[count]; 
        message.points.push_back(des_point);

        m_gait_planning.setPrevPoint(trajectory[count]); 

        m_joint_angle_trajectory_publisher->publish(message);
        RCLCPP_INFO(rclcpp::get_logger("march_gait_planning"), "Stand to sit message published!");
        
        if (count >= (m_gait_planning.getStandToSitGait().size()-1)){
            m_gait_planning.setCounter(m_gait_planning.getStandToSitGait().size()-1); 
        } else {
            m_gait_planning.setCounter(count+1);
        }
    }
}

void GaitPlanningAnglesNode::publishJointTrajectoryPoints(){
    trajectory_msgs::msg::JointTrajectory msg;
    msg.joint_names = {"left_hip_aa", "left_hip_fe", "left_knee", "left_ankle", "right_hip_aa", "right_hip_fe", "right_knee", "right_ankle"};             
    trajectory_msgs::msg::JointTrajectoryPoint trajectory_prev_point; 
    trajectory_msgs::msg::JointTrajectoryPoint trajectory_des_point; 
    initializeConstantsPoints(trajectory_prev_point);
    initializeConstantsPoints(trajectory_des_point); 
    trajectory_des_point.time_from_start.nanosec = int(50*1e6); 

    if (m_gait_planning.getGaitType() == exoState::Walk && m_gait_planning.getPrevGaitType() == exoState::Walk){
        processContinuousWalkingGait(trajectory_prev_point, trajectory_des_point, msg); 
    } else if (m_gait_planning.getGaitType() == exoState::Walk && (m_gait_planning.getPrevGaitType() == exoState::Stand)) {
        processFirstStepWalkingGait(trajectory_prev_point, trajectory_des_point, msg); 
    } else if (m_gait_planning.getGaitType() == exoState::Stand){
        processHomeStandGait(trajectory_prev_point, trajectory_des_point, msg); 
    } else if (m_gait_planning.getGaitType() == exoState::Sit){
        processStandToSitGait(trajectory_prev_point, trajectory_des_point, msg); 
    } else {
        RCLCPP_INFO(rclcpp::get_logger("march_gait_planning"), "Waiting for command"); 
    }
}

int main(int argc, char *argv[]){
    
    rclcpp::init(argc, argv); 

    rclcpp::spin(std::make_shared<GaitPlanningAnglesNode>()); 
    rclcpp::shutdown(); 

    return 0; 
}
