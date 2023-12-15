/*Authors: Femke Buiks and Andrew Hutani, MIX*/

#include "march_gait_planning/gait_planning.hpp"
#include <tuple>
#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream>

struct CSVRow {
    std::string x_swing;
    std::string z_swing;
    std::string x_stance; 
    std::string z_stance; 
    // Add more members as needed
};

GaitPlanning::GaitPlanning()
: m_gait_type(), 
  m_current_stance_foot(), 
  m_current_left_foot_position(), 
  m_current_right_foot_position(), 
  m_bezier_trajectory(), 
  m_step_size()
  {
    std::cout << "Gait Planning Class created" << std::endl; 
    setBezierGait(); 
    std::cout << "Bezier CSV created" << std::endl; 
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

void GaitPlanning::setBezierGait(){
    m_first_step_trajectory = processCSV("src/march_gait_planning/m9_gait_files/cartesian/first_step.csv");
    m_bezier_trajectory = processCSV("src/march_gait_planning/m9_gait_files/cartesian/normal_gait.csv");
}

std::vector<std::array<double, 4>> GaitPlanning::getTrajectory() const{
    return m_current_stance_foot == 0 ? m_first_step_trajectory : m_bezier_trajectory;
}

int GaitPlanning::getCurrentStanceFoot() const {
    return m_current_stance_foot; 
}

std::array<double, 3> GaitPlanning::getCurrentLeftFootPos() const{
    return m_current_left_foot_position; 
}

std::array<double, 3> GaitPlanning::getCurrentRightFootPos() const{
    return m_current_right_foot_position; 
}

exoState GaitPlanning::getGaitType() const{
    return m_gait_type; 
}

std::vector<std::array<double, 4>> GaitPlanning::processCSV(const std::string& filename){
    std::vector<CSVRow> data;
    std::ifstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return {};
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        CSVRow row;

        std::getline(iss, row.x_swing, ',');
        std::getline(iss, row.z_swing, ',');
        std::getline(iss, row.x_stance, ',');
        std::getline(iss, row.z_stance, ','); 

        data.push_back(row);
    }

    file.close();

    std::vector<std::array<double, 4>> trajectory;
    for (const auto& row : data) {
        trajectory.push_back({std::stod(row.x_swing), std::stod(row.z_swing), std::stod(row.x_stance), std::stod(row.z_stance)}); 
    }

    return trajectory;
}