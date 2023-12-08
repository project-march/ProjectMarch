//TODO: Fix the header files
#include "march_gait_planning/test_gait_planning.hpp"
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

TestGaitPlanning::TestGaitPlanning()
: m_gait_type(), 
  m_current_stance_foot(), 
  m_current_left_foot_position(), 
  m_current_right_foot_position(), 
  m_bezier_trajectory(), 
  m_step_size()
  {
    std::cout << "Test Gait Planning Class created" << std::endl; 
    setBezierGait(); 
    std::cout << "Bezier CSV created" << std::endl; 
  }

std::vector<std::array<double, 3>> TestGaitPlanning::getFootEndPositions() const {
    // should be for both swing and stance leg. 
    // define as xyz coordinates, but translate to exo frame 
    std::array<double, 3> final_swing_leg_position; 
    std::array<double, 3> final_stance_leg_position;
    // final array has left foot first
        std::vector<std::array<double, 3>> final_array;  
    if (m_current_stance_foot == 0){
        // from home stand, both legs are together. Right leg will take half a step first. 
        final_swing_leg_position = m_current_right_foot_position; 
        final_swing_leg_position[0] += m_step_size/2.0;
        final_stance_leg_position = m_current_left_foot_position; 
        final_stance_leg_position[0] -= m_step_size/2.0; 
        final_array.push_back(final_stance_leg_position); 
        final_array.push_back(final_swing_leg_position); 
    }
    else if (m_current_stance_foot == -1){
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

void TestGaitPlanning::setStepSize(const double &step_size) {
    m_step_size = step_size; 
}

void TestGaitPlanning::setBezierGait(){
    std::vector<CSVRow> data;
    std::vector<CSVRow> data2; 
    
    //TODO: Change the files for the gaits
    std::ifstream file("src/march_gait_planning/m9_gait_files/first_step.csv");
    std::ifstream file2("src/march_gait_planning/m9_gait_files/normal_gait.csv"); 

    if (!file.is_open() || !file2.is_open()) {
        std::cerr << "Error opening file." << std::endl;
    }

    std::string line;
    std::string line2; 
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        CSVRow row;

        // Assuming the CSV has two columns, modify as needed
        std::getline(iss, row.x_swing, ',');
        std::getline(iss, row.z_swing, ',');
        std::getline(iss, row.x_stance, ',');
        std::getline(iss, row.z_stance, ','); 

        // Add more lines if you have more columns

        data.push_back(row);
    }

    file.close();

    for (const auto& row : data) {
        m_first_step_trajectory.push_back({std::stod(row.x_swing), std::stod(row.z_swing), std::stod(row.x_stance), std::stod(row.z_stance)}); 
    }

    while (std::getline(file2, line2)) {
        std::istringstream iss(line2);
        CSVRow row2;

        // Assuming the CSV has two columns, modify as needed
        std::getline(iss, row2.x_swing, ',');
        std::getline(iss, row2.z_swing, ',');
        std::getline(iss, row2.x_stance, ',');
        std::getline(iss, row2.z_stance, ','); 

        // Add more lines if you have more columns

        data2.push_back(row2);
    }

    file2.close(); 

    for (const auto& row2 : data2) {
        m_bezier_trajectory.push_back({std::stod(row2.x_swing), std::stod(row2.z_swing), std::stod(row2.x_stance), std::stod(row2.z_stance)}); 
    } 
}

std::vector<std::array<double, 4>> TestGaitPlanning::getTrajectory() const{
    if (m_current_stance_foot == 0){
        return m_first_step_trajectory; 
    } else {
        return m_bezier_trajectory; 
    }
}

void TestGaitPlanning::setStanceFoot(const int &new_stance_foot){
    m_current_stance_foot = new_stance_foot;  
}

void TestGaitPlanning::setFootPositions(const std::array<double, 3> &new_left_foot_position, const std::array<double, 3> &new_right_foot_position) { 
    m_current_left_foot_position = new_left_foot_position; 
    m_current_right_foot_position = new_right_foot_position; 
}

void TestGaitPlanning::setGaitType(const exoState &new_gait_type){
    m_gait_type = new_gait_type; 
}

int TestGaitPlanning::getCurrentStanceFoot() const {
    return m_current_stance_foot; 
}


std::array<double, 3> TestGaitPlanning::getCurrentLeftFootPos() const{
    return m_current_left_foot_position; 
}


std::array<double, 3> TestGaitPlanning::getCurrentRightFootPos() const{
    return m_current_right_foot_position; 
}

exoState TestGaitPlanning::getGaitType() const{
    return m_gait_type; 
}
