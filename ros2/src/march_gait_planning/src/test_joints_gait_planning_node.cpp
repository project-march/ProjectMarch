#include "march_gait_planning/test_joints_gait_planning_node.hpp"

using std::placeholders::_1; 
// Make time_to_start a constant variable?
// Make the rot and linear joint ranges configurable as a parameter or in config?
constexpr double rotational_range = 0.3;
constexpr double linear_range = 0.1;


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
    
    m_current_joint_angles_msg->points.push_back(previous_point);
    m_current_joint_angles_msg->points.push_back(current_point);

    m_exo_state_subscriber = create_subscription<march_shared_msgs::msg::ExoStateAndJoint>(
        "current_state", 10, std::bind(&TestJointsGaitPlanningNode::currentStateCallback, this, _1)); 
    
    m_test_joint_trajectory_controller_state_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "joint_trajectory_controller/joint_trajectory", 10);

    m_gait_planning.setGaitType(exoState::BootUp); 

    // If everything goes correctly, there is nothing to publish so immediately a request will be sent. 
    auto timer_callback = std::bind(&TestJointsGaitPlanningNode::timerCallback, this);
    m_timer = this->create_wall_timer(std::chrono::milliseconds(50), timer_callback);

}

void TestJointsGaitPlanningNode::currentStateCallback(const march_shared_msgs::msg::ExoStateAndJoint::SharedPtr msg){
    RCLCPP_INFO(get_logger(), "Received current state: %d", msg->state); 
    m_gait_planning.setGaitType((exoState)msg->state);
    setActuatedJoint(msg->joint.data);
}

void TestJointsGaitPlanningNode::footPositionsPublish(){
    m_current_joint_angles_msg->joint_names.push_back(getActuatedJoint());
    switch (m_gait_planning.getGaitType()){
        case exoState::BootUp: {
            break;
        }

        case exoState::Stand: {
            m_current_trajectory.clear();
            m_current_joint_angles_msg->points[1].positions.push_back(0.0);
            RCLCPP_INFO(rclcpp::get_logger("march_test_gait_planning_node"), "Joint angles assigned");
            m_test_joint_trajectory_controller_state_pub_->publish(*m_current_joint_angles_msg);
            RCLCPP_INFO(rclcpp::get_logger("march_test_gait_planning_node"), "Home stand position published!");
            break;
        }

        case exoState::Walk: {

            if (m_current_trajectory.empty()) {
                //TODO: This gives an error: Mismatch between joint_names (1) and positions (0) at point #0.
                m_current_trajectory = m_gait_planning.getTrajectory();
            }
            else{
                double new_angle = 0.2 * m_current_trajectory.front();
                m_current_trajectory.erase(m_current_trajectory.begin());
                m_current_joint_angles_msg->points[1].positions.push_back(new_angle);
                m_test_joint_trajectory_controller_state_pub_->publish(*m_current_joint_angles_msg);
                RCLCPP_INFO(rclcpp::get_logger("march_test_gait_planning_node"), "Foot positions published!");
                
            }
            break;
        }

    }
    m_current_joint_angles_msg->points[0].positions = m_current_joint_angles_msg->points[1].positions; 
    m_current_joint_angles_msg->points[1].positions.clear();
    m_current_joint_angles_msg->joint_names.clear();
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

