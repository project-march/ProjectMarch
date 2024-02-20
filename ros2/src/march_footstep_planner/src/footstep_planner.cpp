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
    std::sort(m_planes_list.begin(), m_planes_list.end(), [this](const march_shared_msgs::msg::Plane& plane1, const         march_shared_msgs::msg::Plane& plane2){
        return compareDistance(plane1.centroid, plane2.centroid); 
    });
}

march_shared_msgs::msg::Plane& FootstepPlanner::findSafePlane(size_t index){
    // This function recursively iterates through the list of planes ranked by distance. It 
    // returns the first plane that is found to be safe to step on. 
    if (index >= m_planes_list.size()){
        throw std::logic_error( "No safe plane found!");
    }
    if (checkCentroidPlaneSafeDistance(m_planes_list[index])) {
        return m_planes_list[index];
    }
    return findSafePlane(index + 1); 
}

bool FootstepPlanner::checkOverlapPlaneFootbox(const march_shared_msgs::msg::Plane& plane) const {
    // This function should in some way check if an area the size of the two feet around the centroid is safe
    // to step on, aka falls within plane. We might want to check with just one foot, depending
    // on strategy. If it is a circle (stepping stone) the feet will never fully fit, so we need to set a default value of true. 
    if (checkIfCircle(plane)){
        return true; 
    } else {
        bool fits_x = ((plane.centroid.x + m_foot_size[0]/2) < plane.upper_boundary_point.x && 
            (plane.centroid.x - m_foot_size[0]/2) > plane.lower_boundary_point.x); 
        bool fits_y = ((plane.centroid.y + m_foot_size[1]/2) < plane.left_boundary_point.y && 
            (plane.centroid.y - m_foot_size[1]/2) > plane.right_boundary_point.y); 
        return (fits_x && fits_y); 
    }
    // What to do if it doesn't fit???? 
}

bool FootstepPlanner::checkIfCircle(const march_shared_msgs::msg::Plane& plane) const {
    //This function checks whether a given plane is a circle or not by checking radius 
    return (std::abs(plane.upper_boundary_point.x - plane.lower_boundary_point.x - (plane.left_boundary_point.y +           std::abs(plane.right_boundary_point.y))) < 0.05);
}

