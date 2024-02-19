/*
Author: Femke Buiks, MIX
*/

#include "march_footstep_planner/footstep_planner.hpp"
#include <iostream> 

FootstepPlanner::FootstepPlanner()
: m_step_distance_threshold(), 
  m_foot_size(), 
  m_left_foot_position(), 
  m_right_foot_position(), 
  m_planes_list()
  {
    std::cout << "Footstep Planner class created" << std::endl;
    setDistanceThreshold(0.8); 
    setFootSize(30, 25, 0); //z is not yet relevant, this box should include BOTH feet
    std::cout << "Member variables initialized" << std::endl; 
  }

void FootstepPlanner::setFootPositions(const std::array<double, 3> &new_left_foot_position, const std::array<double, 3> &new_right_foot_position) { 
    m_left_foot_position = new_left_foot_position; 
    m_right_foot_position = new_right_foot_position; 
}

void FootstepPlanner::setDistanceThreshold(const double &distance){
    m_step_distance_threshold = distance; 
}

void FootstepPlanner::setFootSize(const double &width, const double &length, const double &height){
    m_foot_size = {width, length, height}; 
}

void FootstepPlanner::setPlanesList(const std::vector<march_shared_msgs::msg::Plane> &planes){
    m_planes_list = planes; 
}

std::array<double, 3> FootstepPlanner::getFootSize() const {
    return m_foot_size; 
}

double FootstepPlanner::getDistanceThreshold() const {
    return m_step_distance_threshold; 
}

std::array<double, 3> FootstepPlanner::getLeftFootPosition() const {
    return m_left_foot_position; 
}

std::array<double, 3> FootstepPlanner::getRightFootPosition() const{
    return m_right_foot_position; 
} 

std::vector<march_shared_msgs::msg::Plane> FootstepPlanner::getPlanesList() const{
    return m_planes_list; 
}

bool FootstepPlanner::compareDistance(const geometry_msgs::msg::Point& plane1_centroid, const geometry_msgs::msg::Point& plane2_centroid) const{
    // this function should compare two planes and return a bool describing if the first plane centroid 
    // is closer to the current foot positions than the second plane centroid 
    double current_x_position = getRightFootPosition()[0]; 
    return ((plane1_centroid.x - current_x_position) < (plane2_centroid.x- current_x_position)); 
}

bool FootstepPlanner::checkCentroidPlaneSafeDistance(const march_shared_msgs::msg::Plane& plane) const{
    // this function should return a bool describing if the centroid of a plane is close enough to 
    // safely reach, given ranges of motion etc. 
    return (plane.centroid.x < (getDistanceThreshold() + getRightFootPosition()[0])); 
}

void FootstepPlanner::rankPlanesByDistance(){
    // this function should sort the list of planes by centroid distance to the current foot poistions, 
    // starting with the closest first 
    std::sort(m_planes_list.begin(), m_planes_list.end(), [this](const march_shared_msgs::msg::Plane& plane1, const march_shared_msgs::msg::Plane& plane2){
        return compareDistance(plane1.centroid, plane2.centroid); 
    });
}

