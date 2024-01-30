#include "march_footstep_planner/include/footstep_planner.hpp"
#include <iostream> 

FootstepPlanner::FootstepPlanner()
: m_step_distance_threshold(), 
  m_foot_size()
  {
    std::cout << "Footstep Planner class created" << std::endl;
    setDistanceThreshold(0.4); 
    setFootSize(15, 25, 0); //z is not yet relevant
    std::cout << "Member variables initialized" << std::endl; 
  }

void FootstepPlanner::setFootPositions(const std::array<double, 3> &new_left_foot_position, const std::array<double, 3> &new_right_foot_position) { 
    m_left_foot_position = new_left_foot_position; 
    m_right_foot_position = new_right_foot_position; 
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

