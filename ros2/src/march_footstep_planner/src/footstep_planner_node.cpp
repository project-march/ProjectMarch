/*
Author: Femke Buiks, MIX
*/

#include "march_footstep_planner/footstep_planner_node.hpp"

using std::placeholders::_1; 

FootstepPlannerNode::FootstepPlannerNode()
: Node("march_footstep_planner_node"), 
  m_footstep_planner(FootstepPlanner()), 
  m_desired_footstep_msg(std::make_shared<march_shared_msgs::msg::FootStepOutput>()), 
  m_gait_type()
{
    m_variable_footstep_publisher = create_publisher<march_shared_msgs::msg::FootStepOutput>("footsteps", 1); 
    m_exo_mode_subscriber = create_subscription<march_shared_msgs::msg::ExoMode>(
        "current_mode", 10, std::bind(&FootstepPlannerNode::currentModeCallback, this, _1)); 
    m_exo_joint_state_subscriber = create_subscription<march_shared_msgs::msg::StateEstimation>(
        "state_estimation/state", 100, std::bind(&FootstepPlannerNode::currentExoStateCallback, this, _1)); 
    m_planes_subscriber = create_subscription<march_shared_msgs::msg::AllPlanes>(
        "planes", 10, std::bind(&FootstepPlannerNode::planesCallback, this, _1)); 
}

void FootstepPlannerNode::currentModeCallback(const march_shared_msgs::msg::ExoMode::SharedPtr msg){
    // A footstep should only be calculated when the gait type is dynamic/cameras. Maybe
    // use a different member variable or method to identify when this is the case. 
    RCLCPP_INFO(this->get_logger(), "Received current mode: %s", toString(static_cast<exoMode>(msg->mode)).c_str()); 
    m_gait_type = (exoMode)msg->mode; 
}

void FootstepPlannerNode::currentExoStateCallback(const march_shared_msgs::msg::StateEstimation::SharedPtr msg) {
    std::array<double, 3> new_left_foot_position = {msg->foot_pose[0].position.x, msg->foot_pose[0].position.y, 
    msg->foot_pose[0].position.z};
    std::array<double, 3> new_right_foot_position = {msg->foot_pose[1].position.x, msg->foot_pose[1].position.y, 
    msg->foot_pose[1].position.z};
    m_footstep_planner.setFootPositions(new_left_foot_position, new_right_foot_position); 
}

void FootstepPlannerNode::planesCallback(const march_shared_msgs::msg::AllPlanes::SharedPtr msg){
    if (msg->planes.empty()) {
        throw std::logic_error("No planes in message!"); 
    } 
    RCLCPP_DEBUG(this->get_logger(), "Received list of planes!");
    m_footstep_planner.setPlanesList(msg->planes);
    if (m_gait_type == exoMode::VariableStep){
        footstepOutputPublish(); 
    }
}

// TODO: somewhere include logic to say whether feet should be placed together (e.g. on stepping stone) or in a normal box (use 
// an offset, as gait planning is determined in how the feet are placed, or send y position to gaitplanning)

// TODO: IKS currently locates ankles of feet at coordinate you send it, so add offset (ankle to midfoot distance to centroid for example)

void FootstepPlannerNode::footstepOutputPublish(){
    //This function should ultimately publish distance/desired stepping point on the footstepoutput topic
    m_footstep_planner.rankPlanesByDistance(); 
    RCLCPP_DEBUG(this->get_logger(), "Planes ranked"); 
    march_shared_msgs::msg::Plane& safe_plane = m_footstep_planner.findSafePlane(); 
    RCLCPP_DEBUG(this->get_logger(), "Safe plane found!"); 
    if (m_footstep_planner.checkOverlapPlaneFootbox(safe_plane)){
        m_desired_footstep_msg->stepping_point = (safe_plane.centroid); 
        m_variable_footstep_publisher->publish(*m_desired_footstep_msg);
        RCLCPP_INFO(this->get_logger(), "Sent footstep message!"); 
    } else {
        RCLCPP_DEBUG(this->get_logger(), "No overlap"); 
    }
}

int main(int argc, char *argv[]){
    
    rclcpp::init(argc, argv); 

    rclcpp::spin(std::make_shared<FootstepPlannerNode>()); 
    rclcpp::shutdown(); 

    return 0; 
}

