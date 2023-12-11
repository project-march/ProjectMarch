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

JointAngleGaitNode::JointAngleGaitNode()
 : Node("joint_angle_gait_node")
{ 
    joint_angle_trajectory_publisher = create_publisher<trajectory_msgs::msg::JointTrajectory>("joint_trajectory_controller/joint_trajectory", 10); 
    std::cout << "Joint trajectory publisher created" << std::endl; 
    getAngleCSV(); 
    std::cout << "Angle file read and saved" << std::endl; 
    auto timer_publish = std::bind(&JointAngleGaitNode::publishJointTrajectory, this);
    m_timer = this->create_wall_timer(std::chrono::milliseconds(50), timer_publish);
    std::cout << "Timer function created" << std::endl; 
}

void JointAngleGaitNode::getAngleCSV(){
    std::vector<CSVRow> data; 
    std::ifstream file("src/march_gait_planning/m9_gait_files/q_data.csv"); 
    if (!file.is_open()){
        std::cerr << "Error opening file" << std::endl; 
    }
    std::string line; 
    while (std::getline(file, line)){
        std::istringstream iss(line); 
        CSVRow row; 
        std::getline(iss, row.left_hip_aa, ','); 
        std::getline(iss, row.left_hip_fe, ',');
        std::getline(iss, row.left_knee, ',');
        std::getline(iss, row.left_ankle, ',');
        std::getline(iss, row.right_hip_aa, ',');
        std::getline(iss, row.right_hip_fe, ',');
        std::getline(iss, row.right_knee, ',');
        std::getline(iss, row.right_ankle, ','); 
        data.push_back(row); 
    }

    file.close(); 

    for (const auto& row : data){
        m_data_gait.push_back({std::stod(row.left_hip_aa), std::stod(row.left_hip_fe), std::stod(row.left_knee), std::stod(row.left_ankle), std::stod(row.right_hip_aa), std::stod(row.right_hip_fe), std::stod(row.right_knee), std::stod(row.right_ankle)}); 
    }

    m_counter = 0; 
    m_prev_point = m_data_gait.at(0); 
}

void JointAngleGaitNode::publishJointTrajectory(){
    if (!m_data_gait.empty()){

        trajectory_msgs::msg::JointTrajectory msg;

        msg.joint_names = {"left_hip_aa", "left_hip_fe", "left_knee", "left_ankle", "right_hip_aa", "right_hip_fe", "right_knee", "right_ankle"}; 

        trajectory_msgs::msg::JointTrajectoryPoint trajectory_prev_point; 
        trajectory_prev_point.positions = m_prev_point; 
        trajectory_prev_point.velocities = {0, 0, 0, 0, 0, 0, 0, 0}; 
        trajectory_prev_point.accelerations = {0, 0, 0, 0, 0, 0, 0, 0}; 
        trajectory_prev_point.effort = {0, 0, 0, 0, 0, 0, 0, 0}; 
        trajectory_prev_point.time_from_start.sec = 0; 
        trajectory_prev_point.time_from_start.nanosec = 0; 
        msg.points.push_back(trajectory_prev_point); 

        trajectory_msgs::msg::JointTrajectoryPoint trajectory_des_point; 
        trajectory_des_point.positions = m_data_gait[m_counter]; 
        trajectory_des_point.velocities = {0, 0, 0, 0, 0, 0, 0, 0}; 
        trajectory_des_point.accelerations = {0, 0, 0, 0, 0, 0, 0, 0}; 
        trajectory_des_point.effort = {0, 0, 0, 0, 0, 0, 0, 0}; 
        trajectory_des_point.time_from_start.sec = 0; 
        trajectory_des_point.time_from_start.nanosec = int(50*1e6); 
        msg.points.push_back(trajectory_des_point);

        m_prev_point = m_data_gait[m_counter];

        joint_angle_trajectory_publisher->publish(msg);
        RCLCPP_INFO(rclcpp::get_logger("march_gait_planning"), "Message published!"); 

        if (m_counter >= (m_data_gait.size()-1)){
            m_counter = 0;
        } else {
            m_counter += 1; 
        }
    }
}

int main(int argc, char *argv[]){
    
    rclcpp::init(argc, argv); 

    rclcpp::spin(std::make_shared<JointAngleGaitNode>()); 
    rclcpp::shutdown(); 

    return 0; 
}
