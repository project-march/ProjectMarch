/*Authors: Andrew Hutani, MIX

This node is used to test the gait planning for each joint seperately; it will send a sinusoidal wave to a single joint while keeping the other nodes in the home stand position.
This node is only called in the test_joints launch file.

*/

#include "march_gait_planning/test_joints_gait_planning_node.hpp"

using std::placeholders::_1; 
// Make time_to_start a constant variable?
// Make the rot and linear joint ranges configurable as a parameter or in config?
constexpr double rotational_range = 0.3;
constexpr double linear_range = 0.1;

std::vector<std::string> all_possible_joints = {"left_hip_aa", "left_hip_fe", "left_knee", "left_ankle", 
                                            "right_hip_aa", "right_hip_fe", "right_knee", "right_ankle"};

TestJointsGaitPlanningNode::TestJointsGaitPlanningNode()
 : Node("march_test_joints_gait_planning_node"), 
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
    
    m_current_joint_angles_msg->joint_names = all_possible_joints;
    m_current_joint_angles_msg->points.push_back(previous_point);
    m_current_joint_angles_msg->points.push_back(current_point);

    m_exo_mode_subscriber = create_subscription<march_shared_msgs::msg::ExoModeAndJoint>(
        "current_mode", 10, std::bind(&TestJointsGaitPlanningNode::currentModeCallback, this, _1)); 
    
    m_test_joint_trajectory_controller_mode_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "joint_trajectory_controller/joint_trajectory", 10);

    m_gait_planning.setGaitType(exoMode::BootUp); 

    // If everything goes correctly, there is nothing to publish so immediately a request will be sent. 
    auto timer_callback = std::bind(&TestJointsGaitPlanningNode::timerCallback, this);
    m_timer = this->create_wall_timer(std::chrono::milliseconds(50), timer_callback);

}

void TestJointsGaitPlanningNode::currentModeCallback(const march_shared_msgs::msg::ExoModeAndJoint::SharedPtr msg){
    RCLCPP_INFO(get_logger(), "Received current mode: %s", toString(static_cast<exoMode>(msg->mode)).c_str()); 
    m_gait_planning.setGaitType((exoMode)msg->mode);
    setActuatedJoint(msg->joint.data);
}

void TestJointsGaitPlanningNode::footPositionsPublish(){
    switch (m_gait_planning.getGaitType()){
        case exoMode::BootUp: {
            break;
        }

        case exoMode::Stand: {
            m_current_trajectory.clear();
            m_current_joint_angles_msg->points[1].positions = {-0.03, 0.042, -0.0, -0.016, -0.03, 0.042, -0.0, -0.016};
            m_current_joint_angles_msg->points[0].positions = {-0.03, 0.042, -0.0, -0.016, -0.03, 0.042, -0.0, -0.016};

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
                //TODO: This does not work for the HAA, since positive is defined to be adduction.
                double new_angle = 0.1* m_current_trajectory.front() + 0.1;
                m_current_trajectory.erase(m_current_trajectory.begin());
                for (size_t i = 0; i < m_current_joint_angles_msg->joint_names.size(); ++i) {
                    if (m_current_joint_angles_msg->joint_names[i] == getActuatedJoint()) {
                        // This is the joint we want to actuate, set the new position
                        m_current_joint_angles_msg->points[1].positions.push_back(new_angle);
                    } else {
                        // This is not the joint we want to actuate, set the current position
                        m_current_joint_angles_msg->points[1].positions.push_back(
                            m_current_joint_angles_msg->points[0].positions[i]);
                    }
                }
                m_test_joint_trajectory_controller_mode_pub_->publish(*m_current_joint_angles_msg);
                RCLCPP_INFO(rclcpp::get_logger("march_test_gait_planning_node"), "Foot positions published!");
            }
            break;
        }

        default : 
            break; 

    }
    m_current_joint_angles_msg->points[0].positions = m_current_joint_angles_msg->points[1].positions; 
    m_current_joint_angles_msg->points[1].positions.clear();
}



void TestJointsGaitPlanningNode::timerCallback() {
    // This code will be executed every 50 milliseconds
    footPositionsPublish();    
}

void TestJointsGaitPlanningNode::setActuatedJoint(const std::string &actuated_joint){
    m_actuated_joint = actuated_joint; 
}

std::string TestJointsGaitPlanningNode::getActuatedJoint() const{
    return m_actuated_joint; 
}

int main(int argc, char *argv[]){
    
    rclcpp::init(argc, argv); 

    rclcpp::spin(std::make_shared<TestJointsGaitPlanningNode>()); 
    rclcpp::shutdown(); 

    return 0; 
}

