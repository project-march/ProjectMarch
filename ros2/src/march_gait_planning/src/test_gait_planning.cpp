#include "march_gait_planning/test_gait_planning.hpp"
#include <iostream>
#include <fstream>

TestGaitPlanning::TestGaitPlanning()
  {
    std::cout << "Test Gait Planning Class created" << std::endl; 
    loadTrajectoryFromCSV(); 
    std::cout << "Bezier CSV created" << std::endl; 
  }

void TestGaitPlanning::loadTrajectoryFromCSV() {
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

std::vector<double> TestGaitPlanning::getTrajectory() const{
    return m_trajectory;
}

void TestGaitPlanning::setGaitType(const exoState &new_gait_type){
    m_gait_type = new_gait_type; 
}

exoState TestGaitPlanning::getGaitType() const{
    return m_gait_type; 
}
