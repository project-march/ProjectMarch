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
}

void GaitPlanningAnglesNode::currentStateCallback(const march_shared_msgs::msg::ExoState::SharedPtr msg){
    RCLCPP_INFO(rclcpp::get_logger("march_gait_planning"), "received current state: %d", msg->state); 
    m_gait_planning.setGaitType((exoState)msg->state); 
}


void GaitPlanningAnglesNode::publishJointTrajectoryPoints(){
    if (m_gait_planning.getGaitType() == exoState::Walk){
        if (!m_gait_planning.getAngleTrajectory().empty()){

        trajectory_msgs::msg::JointTrajectory msg;

        msg.joint_names = {"left_hip_aa", "left_hip_fe", "left_knee", "left_ankle", "right_hip_aa", "right_hip_fe", "right_knee", "right_ankle"}; 

        std::vector<std::vector<double>> trajectory = m_gait_planning.getAngleTrajectory();
        int count = m_gait_planning.getCounter();  

        trajectory_msgs::msg::JointTrajectoryPoint trajectory_prev_point; 
        trajectory_prev_point.positions = m_gait_planning.getPrevPoint(); 
        trajectory_prev_point.velocities = {0, 0, 0, 0, 0, 0, 0, 0}; 
        trajectory_prev_point.accelerations = {0, 0, 0, 0, 0, 0, 0, 0}; 
        trajectory_prev_point.effort = {0, 0, 0, 0, 0, 0, 0, 0}; 
        trajectory_prev_point.time_from_start.sec = 0; 
        trajectory_prev_point.time_from_start.nanosec = 0; 
        msg.points.push_back(trajectory_prev_point); 

        trajectory_msgs::msg::JointTrajectoryPoint trajectory_des_point; 
        trajectory_des_point.positions = trajectory[count]; 
        trajectory_des_point.velocities = {0, 0, 0, 0, 0, 0, 0, 0}; 
        trajectory_des_point.accelerations = {0, 0, 0, 0, 0, 0, 0, 0}; 
        trajectory_des_point.effort = {0, 0, 0, 0, 0, 0, 0, 0}; 
        trajectory_des_point.time_from_start.sec = 0; 
        trajectory_des_point.time_from_start.nanosec = int(50*1e6); 
        msg.points.push_back(trajectory_des_point);

        m_gait_planning.setPrevPoint(trajectory[count]); 

        m_joint_angle_trajectory_publisher->publish(msg);
        RCLCPP_INFO(rclcpp::get_logger("march_gait_planning"), "Message published!"); 

        if (count >= (m_gait_planning.getAngleTrajectory().size()-1)){
            m_gait_planning.setCounter(0); 
        } else {
            m_gait_planning.setCounter(count+1);
        }
    }
    } else {
        RCLCPP_INFO(rclcpp::get_logger("march_gait_planning"), "Waiting to walk"); 
    }
}

int main(int argc, char *argv[]){
    
    rclcpp::init(argc, argv); 

    rclcpp::spin(std::make_shared<GaitPlanningAnglesNode>()); 
    rclcpp::shutdown(); 

    return 0; 
}
