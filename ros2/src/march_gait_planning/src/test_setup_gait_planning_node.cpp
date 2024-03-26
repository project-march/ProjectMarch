#include "march_gait_planning/test_setup_gait_planning_node.hpp"

using std::placeholders::_1; 
// Make time_to_start a constant variable?
// Make the rot and linear joint ranges configurable as a parameter or in config?
constexpr double rotational_range = 0.3;
constexpr double linear_range = 0.1;


TestSetupGaitPlanningNode::TestSetupGaitPlanningNode()
 : Node("march_test_setup_gait_planning_node"), 
   m_gait_planning(TestSetupGaitPlanning()),
   m_current_trajectory(),
   m_current_joint_angles_msg(std::make_shared<trajectory_msgs::msg::JointTrajectory>())
{
    trajectory_msgs::msg::JointTrajectoryPoint previous_point;
    previous_point.positions.push_back(0.0);
    previous_point.time_from_start = rclcpp::Duration(0, 0);

    trajectory_msgs::msg::JointTrajectoryPoint current_point;
    current_point.positions.push_back(0.0);
    current_point.time_from_start = rclcpp::Duration(0, 50000000);  //50 ms
    
    m_current_joint_angles_msg->points.push_back(previous_point);
    m_current_joint_angles_msg->points.push_back(current_point);

    m_exo_mode_subscriber = create_subscription<march_shared_msgs::msg::ExoMode>(
        "current_mode", 10, std::bind(&TestSetupGaitPlanningNode::currentModeCallback, this, _1)); 
    
    m_test_joint_trajectory_controller_mode_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "joint_trajectory_controller/joint_trajectory", 10);

    m_gait_planning.setGaitType(exoMode::BootUp); 

    // If everything goes correctly, there is nothing to publish so immediately a request will be sent. 
    auto timer_callback = std::bind(&TestSetupGaitPlanningNode::timerCallback, this);
    m_timer = this->create_wall_timer(std::chrono::milliseconds(50), timer_callback);

    // Get parameters from parameter server and use that joint
    this->declare_parameter<bool>("test_rotational", true);
    this->get_parameter("test_rotational", m_test_rotational);
    const std::string joint_name = m_test_rotational ? "rotational_joint" : "linear_joint";
    m_current_joint_angles_msg->joint_names.push_back(joint_name);
    RCLCPP_INFO(rclcpp::get_logger("march_test_gait_planning_node"), "%s Joint", m_test_rotational ? "Rotational" : "Linear");
}

void TestSetupGaitPlanningNode::currentModeCallback(const march_shared_msgs::msg::ExoMode::SharedPtr msg){
    RCLCPP_INFO(get_logger(), "Received current mode: %s", toString(static_cast<exoMode>(msg->mode)).c_str()); 
    m_gait_planning.setGaitType((exoMode)msg->mode);
}

void TestSetupGaitPlanningNode::footPositionsPublish(){
    if (m_test_rotational){
        executeRotationalJointGait();
    }
    else{ 
        executeLinearJointGait();
    }
    m_current_joint_angles_msg->points[0].positions = m_current_joint_angles_msg->points[1].positions; 
    m_current_joint_angles_msg->points[1].positions.clear();
}

void TestSetupGaitPlanningNode::executeRotationalJointGait(){

    switch (m_gait_planning.getGaitType()){

        case exoMode::BootUp: {
            break;
        }

        case exoMode::Stand: {

            m_current_trajectory.clear();
            m_current_joint_angles_msg->points[1].positions.push_back(0.0);
            RCLCPP_INFO(rclcpp::get_logger("march_test_gait_planning_node"), "Joint angles assigned");
            m_test_joint_trajectory_controller_mode_pub_->publish(*m_current_joint_angles_msg);
            RCLCPP_INFO(rclcpp::get_logger("march_test_gait_planning_node"), "Home stand position published!");
            break;
        }
        case exoMode::Walk: {

            if (m_current_trajectory.empty()) {
                //TODO: This gives an error: Mismatch between joint_names (1) and positions (0) at point #0.
                m_current_trajectory = m_gait_planning.getTrajectory();
            }
            else{
                double new_angle = rotational_range * m_current_trajectory.front();
                m_current_trajectory.erase(m_current_trajectory.begin());
                m_current_joint_angles_msg->points[1].positions.push_back(new_angle);
                m_test_joint_trajectory_controller_mode_pub_->publish(*m_current_joint_angles_msg);
                RCLCPP_INFO(rclcpp::get_logger("march_test_gait_planning_node"), "Foot positions published!");
                
            }
            break;
        }
        default: {

            //TODO: Connect this to the Safety node
            RCLCPP_INFO(get_logger(), "Unsupported gait type");
            break;
        }
    }
}

void TestSetupGaitPlanningNode::executeLinearJointGait(){

    switch (m_gait_planning.getGaitType()){

        case exoMode::BootUp: {
        break;
        }

        case exoMode::Stand: {

            m_current_trajectory.clear();
            m_current_joint_angles_msg->points[1].positions.push_back(0.0);
            m_test_joint_trajectory_controller_mode_pub_->publish(*m_current_joint_angles_msg);
            RCLCPP_INFO(rclcpp::get_logger("march_test_gait_planning_node"), "Home stand position published!");
            break;
        }
        case exoMode::Walk: {

            if (m_current_trajectory.empty()) {
                m_current_trajectory = m_gait_planning.getTrajectory();
            }
            else{
                double new_angle = linear_range * m_current_trajectory.front();
                m_current_trajectory.erase(m_current_trajectory.begin());
                m_current_joint_angles_msg->points[1].positions.push_back(new_angle);
                m_test_joint_trajectory_controller_mode_pub_->publish(*m_current_joint_angles_msg);
                RCLCPP_INFO(rclcpp::get_logger("march_test_gait_planning_node"), "Foot positions published!");
            }
            break;
        }
        default: {

            //TODO: Connect this to the Safety node
            RCLCPP_INFO(get_logger(), "Unsupported gait type");
            break;
        }
    }
}

void TestSetupGaitPlanningNode::timerCallback() {
    // This code will be executed every 50 milliseconds
    footPositionsPublish();    
}

int main(int argc, char *argv[]){
    
    rclcpp::init(argc, argv); 

    rclcpp::spin(std::make_shared<TestSetupGaitPlanningNode>()); 
    rclcpp::shutdown(); 

    return 0; 
}

