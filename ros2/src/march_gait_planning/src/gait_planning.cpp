#include "march_gait_planning/gait_planning.hpp"
#include <tuple>
#include <cmath>
#include <iostream>

//TODO: create global variable number of timesteps in a step, find out how to calculate this, maybe 
// include joint velocities?  

int NUMBER_OF_TIME_STEPS = 1000; 

GaitPlanning::GaitPlanning()
: m_gait_type(), 
  m_current_stance_foot(), 
  m_current_left_foot_position(), 
  m_current_right_foot_position(), 
  m_path_to_yaml(), 
  m_bezier_trajectory(), 
  m_step_size()
  {
    std::cout << "Gait Planning Class created" << std::endl; 
  }

std::vector<std::array<double, 3>> GaitPlanning::getFootEndPositions() const {
    // should be for both swing and stance leg. 
    // define as xyz coordinates, but translate to exo frame 
    std::array<double, 3> final_swing_leg_position; 
    std::array<double, 3> final_stance_leg_position;
    // final array has left foot first
        std::vector<std::array<double, 3>> final_array;  
    if (m_current_stance_foot == 2){
        // from home stand, both legs are together. Right leg will take half a step first. 
        final_swing_leg_position = m_current_right_foot_position; 
        final_swing_leg_position[0] += m_step_size/2.0;
        final_stance_leg_position = m_current_left_foot_position; 
        final_stance_leg_position[0] -= m_step_size/2.0; 
        final_array.push_back(final_stance_leg_position); 
        final_array.push_back(final_swing_leg_position); 
    }
    else if (m_current_stance_foot == 0){
        // left stance foot, right is assumed to be behind left foot. 
        final_swing_leg_position = m_current_right_foot_position; 
        final_swing_leg_position[0] += m_step_size;
        final_stance_leg_position = m_current_left_foot_position; 
        final_stance_leg_position[0] -= m_step_size;
        final_array.push_back(final_stance_leg_position); 
        final_array.push_back(final_swing_leg_position); 
    }
    else if (m_current_stance_foot == 1){
        // right stand foot, left is assumed to be behind left foot. 
        final_swing_leg_position = m_current_left_foot_position; 
        final_swing_leg_position[0] += m_step_size;
        final_stance_leg_position = m_current_right_foot_position; 
        final_stance_leg_position[0] -= m_step_size;
        final_array.push_back(final_swing_leg_position); 
        final_array.push_back(final_stance_leg_position); 
    }
    return final_array; 
}

//TODO: create function that publishes the bezier curve points per time point. 
// This function should also publish the stance foot movement to final position in linear steps. 

void GaitPlanning::setStepSize(const double &step_size) {
    m_step_size = step_size; 
}

void GaitPlanning::createBezierTrajectory(){
    //TODO: optimize to prevent for loop for every time step
    std::pair<double, double> p0(0, 0);
    std::pair<double, double> p1(0.05, 0.12);
    std::pair<double, double> p2(m_step_size / 3, 0.12);
    std::pair<double, double> p3(m_step_size, 0);
    std::vector<std::array<double, 3>> points;
    double z; 
    for (int i = 0; i <= NUMBER_OF_TIME_STEPS; ++i) {
        if (m_current_stance_foot){
        z = m_current_left_foot_position[2];  
        }
        else {
        z = m_current_right_foot_position[2]; 
        }
        double t = static_cast<double>(i) / NUMBER_OF_TIME_STEPS;
        double x = pow(1 - t, 3) * p0.first + 3 * pow(1 - t, 2) * t * p1.first + 3 * (1 - t) * pow(t, 2) * p2.first + pow(t, 3) * p3.first;
        double y = pow(1 - t, 3) * p0.second + 3 * pow(1 - t, 2) * t * p1.second + 3 * (1 - t) * pow(t, 2) * p2.second + pow(t, 3) * p3.second;
        points.push_back({x, y, z});
    }
    m_bezier_trajectory = points;
}

void GaitPlanning::setStanceFoot(const int &new_stance_foot){
    m_current_stance_foot = new_stance_foot; 
}

void GaitPlanning::setFootPositions(const std::array<double, 3> &new_left_foot_position, const std::array<double, 3> &new_right_foot_position) { 
    m_current_left_foot_position = new_left_foot_position; 
    m_current_right_foot_position = new_right_foot_position; 
}

void GaitPlanning::setGaitType(const exoState &new_gait_type){
    m_gait_type = new_gait_type; 
}

void GaitPlanning::setPathToYaml(const std::string &path_to_yaml){
    m_path_to_yaml = path_to_yaml; 
}




