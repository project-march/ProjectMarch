/*
Author: Femke Buiks, MIX
*/

#include "march_footstep_planner/footstep_planner_node.hpp"

using std::placeholders::_1; 

FootstepPlannerNode::FootstepPlannerNode()
: Node("march_footstep_planner_node"), 
  m_footstep_planner(FootstepPlanner()), 
  m_desired_footstep_msg(std::make_shared<march_shared_msgs::msg::FootStepOutput>()), 
  m_planes_list(), 
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
    RCLCPP_INFO(this->get_logger(), "Received current mode: %d", msg->mode); 
    m_gait_type = (exoMode)msg->mode; 
    // if (m_gait_type == exoMode::VariableWalk){
    //     footstepOutputPublish(); 
    // }
    // Maybe this function is not necessary, as this logic of only sending distances when receiving mode 
    // command will be done in the vision module. Then we only need to include this type of logic in the 
    // planes callback. 
}

void FootstepPlannerNode::currentExoStateCallback(const march_shared_msgs::msg::StateEstimation::SharedPtr msg) {
    std::array<double, 3> new_left_foot_position = {msg->foot_pose[0].position.x, msg->foot_pose[0].position.y, msg->foot_pose[0].position.z};
    std::array<double, 3> new_right_foot_position = {msg->foot_pose[1].position.x, msg->foot_pose[1].position.y, msg->foot_pose[1].position.z};
    m_footstep_planner.setFootPositions(new_left_foot_position, new_right_foot_position); 
}

void FootstepPlannerNode::planesCallback(const march_shared_msgs::msg::AllPlanes::SharedPtr msg){
    RCLCPP_INFO(this->get_logger(), "Received list of planes!"); 
    m_planes_list = msg->planes; 
    if (m_gait_type == exoMode::VariableWalk){
        footstepOutputPublish(); 
    }
}

bool FootstepPlannerNode::compareDistance(const march_shared_msgs::msg::Plane& plane1, const march_shared_msgs::msg::Plane& plane2) const{
    // this function should compare two planes and return a bool describing if the first plane centroid 
    // is closer to the current foot positions than the second plane centroid 
    double current_x_position = m_footstep_planner.getRightFootPosition()[0]; 
    return ((plane1.centroid.x - current_x_position) < (plane2.centroid.x- current_x_position)); 
}

void FootstepPlannerNode::rankPlanesByDistance(){
    // this function should sort the list of planes by centroid distance to the current foot poistions, 
    // starting with the closest first 
    // Maybe do this recursively??????

    // std::sort(m_planes_list.begin(), m_planes_list.end(), compareDistance); 

    // ChatGPT implementation: 
    std::sort(m_planes_list.begin(), m_planes_list.end(), [this](const march_shared_msgs::msg::Plane& plane1, const march_shared_msgs::msg::Plane& plane2){
        return compareDistance(plane1, plane2); 
    });
}

march_shared_msgs::msg::Plane* FootstepPlannerNode::findSafePlane(size_t index){
    // This function recursively iterates through the list of planes ranked by distance. It 
    // returns the first plane that is found to be safe to step on. checkCentroidPlaneSafe
    if (index >= m_planes_list.size()){
        return nullptr; 
    }
    if (checkCentroidPlaneSafeDistance(m_planes_list[index])) {
        return &m_planes_list[index];
    }
    return findSafePlane(index + 1); 
}

bool FootstepPlannerNode::checkCentroidPlaneSafeDistance(const march_shared_msgs::msg::Plane& plane) const{
    // this function should return a bool describing if the centroid of a plane is close enough to 
    // safely reach, given ranges of motion etc. 
    return (plane.centroid.x < (m_footstep_planner.getDistanceThreshold()+m_footstep_planner.getRightFootPosition()[0])); 
}

bool FootstepPlannerNode::checkIfCircle(const march_shared_msgs::msg::Plane &plane) const {
    //This function checks whether a given plane is a circle or not
    float x = plane.upper_boundary_point.x - plane.lower_boundary_point.x; 
    float y = plane.left_boundary_point.y + abs(plane.right_boundary_point.y);
    return (x - y < 0.05);  
}

bool FootstepPlannerNode::checkOverlapPlaneFootbox(const march_shared_msgs::msg::Plane& plane) const {
    // This function should in some way check if an area the size of the two feet around the centroid is safe
    // to step on, aka falls within plane. We might want to check with just one foot, depending
    // on strategy.
    if (checkIfCircle(plane)){
        return true; 
    } else {
        // right now, assuming the plane coordinates are with respect to the backpack frame 
        bool fits_x = ((plane.centroid.x + m_footstep_planner.getFootSize()[0]/2) < plane.upper_boundary_point.x && (plane.centroid.x - m_footstep_planner.getFootSize()[0]/2) > plane.lower_boundary_point.x); 
        bool fits_y = ((plane.centroid.y + m_footstep_planner.getFootSize()[1]/2) < plane.left_boundary_point.y && (plane.centroid.y - m_footstep_planner.getFootSize()[1]/2) > plane.right_boundary_point.y); 
        return (fits_x && fits_y); 
    }
}

// somewhere include logic to say whether feet should be placed together or in a normal box (use 
// an offset, as gait planning is determined in how the feet are placed, or send y position to gaitplanning)

void FootstepPlannerNode::footstepOutputPublish(){
    //This function should ultimately publish distance/desired stepping point on the footstepoutput topic
    rankPlanesByDistance(); 
    RCLCPP_INFO(this->get_logger(), "Planes ranked"); 
    march_shared_msgs::msg::Plane* safe_plane = findSafePlane(); 
    if (checkOverlapPlaneFootbox(*safe_plane)){
        m_desired_footstep_msg->distance = (safe_plane->centroid.x - m_footstep_planner.getRightFootPosition()[0]); 
        m_variable_footstep_publisher->publish(*m_desired_footstep_msg);
        RCLCPP_INFO(this->get_logger(), "Sent footstep message! %f", m_desired_footstep_msg->distance); 
    }
}

int main(int argc, char *argv[]){
    
    rclcpp::init(argc, argv); 

    rclcpp::spin(std::make_shared<FootstepPlannerNode>()); 
    rclcpp::shutdown(); 

    return 0; 
}

