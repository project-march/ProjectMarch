#include "march_gait_planning/test_setup_gait_planning.hpp"
#include <iostream>
#include <fstream>

TestSetupGaitPlanning::TestSetupGaitPlanning()
  {
    std::cout << "Test Gait Planning Class created" << std::endl; 
    loadTrajectoryFromCSV(); 
    std::cout << "Bezier CSV created" << std::endl; 
  }

void TestSetupGaitPlanning::loadTrajectoryFromCSV() {
    std::ifstream file("src/march_gait_planning/m9_gait_files/test_trajectory.csv");
    if (!file) {
        std::cerr << "Error opening file." << std::endl;
        return;
    }

    double angle;
    while (file >> angle) {
        m_trajectory.push_back(angle);
    }
}

std::vector<double> TestSetupGaitPlanning::getTrajectory() const{
    return m_trajectory;
}

void TestSetupGaitPlanning::setGaitType(const ExoMode &new_gait_type){
    m_gait_type = new_gait_type; 
}

ExoMode TestSetupGaitPlanning::getGaitType() const{
    return m_gait_type; 
}

void TestSetupGaitPlanning::setPrevPoint(const std::vector<double> &point){
    m_prev_point = point; 
}

std::vector<double> TestSetupGaitPlanning::getPrevPoint() const{
    return m_prev_point; 
}