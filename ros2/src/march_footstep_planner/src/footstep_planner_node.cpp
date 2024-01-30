#include "march_footstep_planner/include/footstep_planner_node.hpp"

using std::placeholders::_1; 

FootstepPlannerNode::FootstepPlannerNode()
: Node("march_footstep_planner_node"), 
  m_footstep_planner(FootstepPlanner()), 
  m_desired_footstep_msg(std::make_shared<march_shared_msgs::msg::FootStepOutput>())
{
    m_variable_footstep_publisher = create_publisher<march_shared_msgs::msg::FootStepOutput>("footsteps", 1); 
    m_exo_mode_subscriber = create_subscription<march_shared_msgs::msg::ExoMode>(
        "current_mode", 10, std::bind(&FootstepPlannerNode::currentModeCallback, this, _1)); 
    m_exo_joint_state_subscriber = create_subscription<march_shared_msgs::msg::StateEstimation>(
        "state_estimation/state", 100, std::bind(&FootstepPlannerNode::currentExoStateCallback, this, _1)); 
    m_planes_subscriber = create_subscription<march_shared_msgs::msg::Planes>(
        "planes", 10, std::bind(&FootstepPlannerNode::planesCallback, this, _1)); 
}

void FootstepPlannerNode::currentModeCallback(const march_shared_msgs::msg::ExoMode::SharedPtr msg){
    // A footstep should only be calculated when the gait type is dynamic/cameras. Maybe
    // use a different member variable or method to identify when this is the case. 
    RCLCPP_INFO(this->get_logger(), "Received current mode: %d", msg->mode); 
    m_gait_type = (exoMode)msg->mode; 
    if (m_gait_type == exoMode::Cameras){
        footstepOutputPublish(); 
    }
}

void FootstepPlannerNode::currentExoStateCallback(const march_shared_msgs::msg::StateEstimation::SharedPtr msg) {
    std::array<double, 3> new_left_foot_position = {msg->foot_pose[0].position.x, msg->foot_pose[0].position.y, msg->foot_pose[0].position.z};
    std::array<double, 3> new_right_foot_position = {msg->foot_pose[1].position.x, msg->foot_pose[1].position.y, msg->foot_pose[1].position.z};
    m_footstep_planner.setFootPositions(new_left_foot_position, new_right_foot_position); 
}

void FootstepPlannerNode::planesCallback(const march_shared_msgs::msg::Planes::SharedPtr msg){
    m_planes_list = msg->planes; 
}

bool FootstepPlannerNode::compareDistance(const plane1, const plane2){
    // this function should compare two planes and return a bool describing if the first plane centroid 
    // is closer to the current foot positions than the second plane centroid 
    double current_x_position = m_footstep_planner.getRightFootPosition()[0]; 
    return ((plane1.centroid.x - current_x_position) < (plane2.centroid.x - current_x_position)); 
}

void FootstepPlannerNode::rankPlanesByDistance(){
    // this function should sort the list of planes by centroid distance to the current foot poistions, 
    // starting with the closest first 
    sort(m_planes_list.begin(), m_planes_list.end(), compareDistance); 
}

bool FootstepPlannerNode::checkCentroidPlaneSafe(const plane){
    // this function should return a bool describing if the centroid of a plane is close enough to 
    // safely reach, given ranges of motion etc. 
    return (plane.centroid.x < (m_footstep_planner.getDistanceThreshold()+m_footstep_planner.getRightFootPosition()[0])); 
}

void FootstepPlannerNode::filterUnsafePlanes(){
    //This function should remove planes that have a centroid too far away to be safe to step on
}

int FootstepPlannerNode::checkNumberOfOverlappingPointsPlaneFootbox() {
    // This function should in some way check if an area the size of the foot around the centroid is safe
    // to step on, aka falls within plane. We might want to check two feet next to each other, depending
    // on strategy. 
}
