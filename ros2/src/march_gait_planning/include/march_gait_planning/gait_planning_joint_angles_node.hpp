
#include "rclcpp/rclcpp.hpp" 
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include <vector>
#include <array> 
#include <iostream> 
#include <fstream> 
#include <sstream> 

class JointAngleGaitNode:public rclcpp::Node {
    public: 
    explicit JointAngleGaitNode(); 

    private: 
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_angle_trajectory_publisher; 

    void publishJointTrajectory(); 
    void getAngleCSV(); 
  
    std::vector<std::vector<double>> m_data_gait; 
    std::vector<double> m_prev_point; 
    int m_counter; 
    rclcpp::TimerBase::SharedPtr m_timer;
};
