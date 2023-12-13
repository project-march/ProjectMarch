#include "march_gait_planning/test_gait_planning_node.hpp"

using std::placeholders::_1; 

TestGaitPlanningNode::TestGaitPlanningNode()
 : Node("march_test_gait_planning_node"), 
   m_gait_planning(TestGaitPlanning()),
   m_current_trajectory(),
   m_current_joint_angles_msg(std::make_shared<trajectory_msgs::msg::JointTrajectory>())
{
    trajectory_msgs::msg::JointTrajectoryPoint previous_point;
    previous_point.positions.push_back(0.0);
    previous_point.time_from_start = rclcpp::Duration(0, 0);

    trajectory_msgs::msg::JointTrajectoryPoint current_point;
    current_point.positions.push_back(0.0);
    current_point.time_from_start = rclcpp::Duration(0, 50000000); //50 ms

    // m_current_joint_angles_msg->points.push_back([]);
    m_current_joint_angles_msg->points.push_back(previous_point);
    m_current_joint_angles_msg->points.push_back(current_point);

    m_exo_state_subscriber = create_subscription<march_shared_msgs::msg::ExoState>(
        "current_state", 10, std::bind(&TestGaitPlanningNode::currentStateCallback, this, _1)); 
    
    m_test_joint_trajectory_controller_state_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "joint_trajectory_controller/joint_trajectory", 10);

    m_gait_planning.setGaitType(exoState::BootUp); // make service between gait planning and state machine for gait type

    // If everything goes correctly, there is nothing to publish so immediately a request will be sent. 
    auto timer_callback = std::bind(&TestGaitPlanningNode::timerCallback, this);
    m_timer = this->create_wall_timer(std::chrono::milliseconds(50), timer_callback);

    this->declare_parameter<bool>("test_rotational", true);
    this->get_parameter("test_rotational", m_test_rotational);

    RCLCPP_INFO(rclcpp::get_logger("march_test_gait_planning_node"), "Test rotational is: %s", m_test_rotational ? "true" : "false");

    if (m_test_rotational){ 
        m_current_joint_angles_msg->joint_names.push_back("rotational_joint");
        RCLCPP_INFO(rclcpp::get_logger("march_test_gait_planning_node"), "Rotational Joint");
    }
    else {
        m_current_joint_angles_msg->joint_names.push_back("linear_joint");
        RCLCPP_INFO(rclcpp::get_logger("march_test_gait_planning_node"), "Linear Joint");
    }
}

void TestGaitPlanningNode::currentStateCallback(const march_shared_msgs::msg::ExoState::SharedPtr msg){
    RCLCPP_INFO(get_logger(), "Received current state: %d", msg->state); 
    m_gait_planning.setGaitType((exoState)msg->state);
}

void TestGaitPlanningNode::footPositionsPublish(){
    if (m_test_rotational){ //Use rotational joint -0.3 to 0.3
        if (m_gait_planning.getGaitType() == exoState::Stand){
            m_current_trajectory.clear();
            m_current_joint_angles_msg->points[1].positions.push_back(0.0);
            RCLCPP_INFO(rclcpp::get_logger("march_test_gait_planning_node"), "Joint angles assigned");
            m_test_joint_trajectory_controller_state_pub_->publish(*m_current_joint_angles_msg);
            RCLCPP_INFO(rclcpp::get_logger("march_test_gait_planning_node"), "Home stand position published!");
        }
        else if (m_gait_planning.getGaitType() == exoState::Walk){
            if (m_current_trajectory.empty()) {
                // This gives an error: Mismatch between joint_names (1) and positions (0) at point #0.
                m_current_trajectory = m_gait_planning.getTrajectory();
            }
            else{
                double new_angle = 0.3*m_current_trajectory.front();
                m_current_trajectory.erase(m_current_trajectory.begin());
                m_current_joint_angles_msg->points[1].positions.push_back(new_angle);
                m_test_joint_trajectory_controller_state_pub_->publish(*m_current_joint_angles_msg);
                RCLCPP_INFO(rclcpp::get_logger("march_test_gait_planning_node"), "Foot positions published!");
            }
        }
    }
    else{ //Use linear joint -0.1 to 0.1
        if (m_gait_planning.getGaitType() == exoState::Stand){
            m_current_trajectory.clear();
            m_current_joint_angles_msg->points[1].positions.push_back(0.0);
            m_test_joint_trajectory_controller_state_pub_->publish(*m_current_joint_angles_msg);
            RCLCPP_INFO(rclcpp::get_logger("march_test_gait_planning_node"), "Home stand position published!");
        }
        else if (m_gait_planning.getGaitType() == exoState::Walk){
            if (m_current_trajectory.empty()) {
                m_current_trajectory = m_gait_planning.getTrajectory();
            }
            else{
                double new_angle = 0.05*m_current_trajectory.front();
                m_current_trajectory.erase(m_current_trajectory.begin());
                m_current_joint_angles_msg->points[1].positions.push_back(new_angle);
                m_test_joint_trajectory_controller_state_pub_->publish(*m_current_joint_angles_msg);
                RCLCPP_INFO(rclcpp::get_logger("march_test_gait_planning_node"), "Foot positions published!");
            }
        }
    }
    m_current_joint_angles_msg->points[0].positions = m_current_joint_angles_msg->points[1].positions; //Set current point as previous point for next iteration
    m_current_joint_angles_msg->points[1].positions.clear();
}

void TestGaitPlanningNode::timerCallback() {
    // This code will be executed every 50 milliseconds
    footPositionsPublish();    
}

int main(int argc, char *argv[]){
    
    rclcpp::init(argc, argv); 

    rclcpp::spin(std::make_shared<TestGaitPlanningNode>()); 
    rclcpp::shutdown(); 

    return 0; 
}

