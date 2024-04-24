/*Authors: Andrew Hutani, MIX

This node is used to test the gait planning for each joint seperately; it will send a sinusoidal wave to a single joint while keeping the other nodes in the home stand position.
This node is only called in the test_joints launch file.

*/

#include "march_gait_planning/test_joints_gait_planning_node.hpp"

using std::placeholders::_1; 

constexpr double ROTATION_RANGE = 0.1;
int INTERPOLATING_TIMESTEPS = 40;

std::vector<std::string> all_possible_joints = {"left_ankle", "left_hip_aa", "left_hip_fe", "left_knee",
                                                "right_ankle", "right_hip_aa", "right_hip_fe", "right_knee"};

std::unordered_map<std::string, int> joint_to_int = {
    {"left_ankle", 0},
    {"left_hip_aa", 1},
    {"left_hip_fe", 2},
    {"left_knee", 3},
    {"right_ankle", 4},
    {"right_hip_aa", 5},
    {"right_hip_fe", 6},
    {"right_knee", 7}
};

TestJointsGaitPlanningNode::TestJointsGaitPlanningNode()
 : Node("march_test_joints_gait_planning_node"), 
   m_gait_planning(TestSetupGaitPlanning()),
   m_current_trajectory(),
   m_joints_msg(),
   m_first_stand(true)
{    
    m_current_state_subscriber = create_subscription<march_shared_msgs::msg::StateEstimation>("state_estimation/state", 10, std::bind(&TestJointsGaitPlanningNode::currentJointAnglesCallback, this, _1)); 
    m_exo_mode_subscriber = create_subscription<march_shared_msgs::msg::ExoModeAndJoint>("current_mode", 10, std::bind(&TestJointsGaitPlanningNode::currentModeCallback, this, _1)); 
    // m_joint_angle_trajectory_publisher = create_publisher<trajectory_msgs::msg::JointTrajectory>("joint_trajectory_controller/joint_trajectory", 10); 
    m_joint_angle_trajectory_publisher = create_publisher<std_msgs::msg::Float64MultiArray>("march_joint_position_controller/commands", 10); 
    RCLCPP_INFO(rclcpp::get_logger("march_gait_planning"), "Joint trajectory publisher created"); 

    m_gait_planning.setGaitType(exoMode::BootUp); 

    m_home_stand = {-0.016, -0.03, 0.042, -0.0, -0.016, -0.03, 0.042, -0.0}; // This is already in alphabetical order

}

void TestJointsGaitPlanningNode::currentModeCallback(const march_shared_msgs::msg::ExoModeAndJoint::SharedPtr msg){
    RCLCPP_INFO(get_logger(), "Received current mode: %s", toString(static_cast<exoMode>(msg->mode)).c_str()); 
    m_gait_planning.setGaitType((exoMode)msg->mode);
    setActuatedJoint(msg->joint.data);
}

void TestJointsGaitPlanningNode::footPositionsPublish(){
    static int counter = 0 ;
    // RCLCPP_INFO(rclcpp::get_logger("march_test_gait_planning_node"), "Current mode: %s", toString(m_gait_planning.getGaitType()).c_str());
    switch (m_gait_planning.getGaitType()){
        case exoMode::BootUp: {
            counter = 0;
            break;
        }

        case exoMode::Stand: {
            processHomeStandGait(counter);
            counter++;
            break;
        }

        case exoMode::Walk: {
            counter = 0;
            if (m_current_trajectory.empty()) {
                //TODO: This gives an error: Mismatch between joint_names (1) and positions (0) at point #0.
                m_current_trajectory = m_gait_planning.getTrajectory();
            }
            else{
                double new_angle = ROTATION_RANGE* m_current_trajectory.front();
                
                if (m_actuated_joint == 1 || m_actuated_joint == 5) {
                    new_angle = -new_angle;
                }

                m_current_trajectory.erase(m_current_trajectory.begin());
                m_joints_msg.data = {};
                for (int i = 0; i < 8; i++) {
                    if (i == getActuatedJoint()) {
                        // This is the joint we want to actuate, set the new position
                        m_joints_msg.data.push_back(new_angle + m_home_stand[i]);
                    } else {
                        // This is not the joint we want to actuate, set the current position
                        m_joints_msg.data.push_back(m_home_stand[i]);
                    }
                }
                m_joint_angle_trajectory_publisher->publish(m_joints_msg);
                RCLCPP_DEBUG(rclcpp::get_logger("march_test_gait_planning_node"), "Foot positions published!");
            }
            break;
        }

        default : 
            break; 

    }
}


void TestJointsGaitPlanningNode::setActuatedJoint(const std::string &actuated_joint){
    if (joint_to_int.find(actuated_joint) != joint_to_int.end()) {
        m_actuated_joint = joint_to_int[actuated_joint]; 
    } else {
        // Handle case where actuated_joint is not a valid joint name
    }
}

int TestJointsGaitPlanningNode::getActuatedJoint() const{
    return m_actuated_joint; 
}

void TestJointsGaitPlanningNode::currentJointAnglesCallback(const march_shared_msgs::msg::StateEstimation::SharedPtr msg) {
    if (m_first_stand && m_gait_planning.getGaitType() == exoMode::Stand) {
        std::vector<double> point = msg->joint_state.position; 
        if (point.size() >= 8) {
            m_gait_planning.setPrevPoint(point); // NOTE: also already in alphabetical order
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
    m_gait_planning.setPrevPoint(msg->joint_state.position);
    footPositionsPublish(); 
    
}

void TestJointsGaitPlanningNode::processHomeStandGait(int counter){
    static std::vector<double> m_initial_point;
    static std::vector<double> m_incremental_steps_to_home_stand;
    if (counter == 0){ // When switching to homestand
        m_incremental_steps_to_home_stand.clear();
        for (unsigned i = 0; i < 8; ++i) {
                m_incremental_steps_to_home_stand.push_back((m_home_stand[i] - m_gait_planning.getPrevPoint()[i]) / INTERPOLATING_TIMESTEPS); // 40 iterations to reach the target, i.e. in 2 seconds
        }
        m_initial_point = m_gait_planning.getPrevPoint();
        RCLCPP_DEBUG(rclcpp::get_logger("march_gait_planning"), "Increments correctly calculated!");
    }
    std::vector<double> temp_moving_to_home_stand;
   
    if (counter < INTERPOLATING_TIMESTEPS){
        RCLCPP_DEBUG(rclcpp::get_logger("march_gait_planning"), "Moving towards home stand!");
        for (unsigned i = 0; i < 8; ++i) {
            m_initial_point[i] += m_incremental_steps_to_home_stand[i];
            temp_moving_to_home_stand.push_back(m_initial_point[i]);
        } 
    }

    else{
        RCLCPP_DEBUG(rclcpp::get_logger("march_gait_planning"), "Home stand position reached!");
        temp_moving_to_home_stand = m_home_stand;
        m_initial_point.clear();
    }
    m_joints_msg.data = temp_moving_to_home_stand;
    m_joint_angle_trajectory_publisher->publish(m_joints_msg);
}

int main(int argc, char *argv[]){
    
    rclcpp::init(argc, argv); 

    rclcpp::spin(std::make_shared<TestJointsGaitPlanningNode>()); 
    rclcpp::shutdown(); 

    return 0; 
}

