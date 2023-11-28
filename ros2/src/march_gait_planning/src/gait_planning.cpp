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
  m_bezier_trajectory()
  {
    std::cout << "Gait Planning Class created" << std::endl; 
  }

std::vector<double> GaitPlanning::getFootEndPositions() const {
    // get stance leg, we need to find foot end positiosn for swing leg only! 
    // import yaml 
    // get data from yaml 
    // store data in vector<double> 
}

double GaitPlanning::getStepSize() const{
    std::vector<double> next_foot_position = getFootEndPositions(); 
    // TODO: define CLEARLY if 1/0 is l/r, now assuming 1 = right and 0 = left 
    // Now, stance foot is defined as right, thus swing leg is LEFT and step size should be for left foot.  
    double step_size; 
    if (m_current_stance_foot){
        step_size = next_foot_position[0] - m_current_left_foot_position[0];  
    }
    else {
        step_size = next_foot_position[0] - m_current_right_foot_position[0]; 
    }
    return step_size;
}

void GaitPlanning::createBezierTrajectory(){
    //TODO: optimize to prevent for loop for every time step
    double step_size = getStepSize(); 
    std::pair<double, double> p0(0, 0);
    std::pair<double, double> p1(0.05, 0.12);
    std::pair<double, double> p2(step_size / 3, 0.12);
    std::pair<double, double> p3(step_size, 0);
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

void GaitPlanning::setFootPositions(const int &new_stance_foot, const std::array<double, 3> &new_left_foot_position, const std::array<double, 3> &new_right_foot_position) {
    m_current_stance_foot = new_stance_foot; 
    m_current_left_foot_position = new_left_foot_position; 
    m_current_right_foot_position = new_right_foot_position; 
}

void GaitPlanning::setGaitType(const exoState &new_gait_type){
    m_gait_type = new_gait_type; 
}

void GaitPlanning::setPathToYaml(const std::string &path_to_yaml){
    m_path_to_yaml = path_to_yaml; 
}




