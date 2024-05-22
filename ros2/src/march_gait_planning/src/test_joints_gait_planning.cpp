#include "march_gait_planning/test_joints_gait_planning.hpp"
#include <iostream>
#include <fstream>

TestJointsGaitPlanning::TestJointsGaitPlanning()
  {
    std::cout << "Test Gait Planning Class created" << std::endl; 
    loadTrajectoryFromCSV("src/march_gait_planning/m9_gait_files/test_trajectory_adpf.csv", m_trajectory_adpf); 
    loadTrajectoryFromCSV("src/march_gait_planning/m9_gait_files/test_trajectory_haa.csv", m_trajectory_haa); 
    loadTrajectoryFromCSV("src/march_gait_planning/m9_gait_files/test_trajectory_hfe.csv", m_trajectory_hfe); 
    loadTrajectoryFromCSV("src/march_gait_planning/m9_gait_files/test_trajectory_kfe.csv", m_trajectory_kfe); 
    std::cout << "Bezier CSV created" << std::endl; 
  }

void TestJointsGaitPlanning::loadTrajectoryFromCSV(const std::string &path, std::vector<double> &member_variable) {
    std::ifstream file(path);
    if (!file) {
        std::cerr << "Error opening file." << std::endl;
    }
    double angle;
    while (file >> angle) {
        member_variable.push_back(angle);
    }
}

std::vector<double> TestJointsGaitPlanning::getTrajectory(int& actuated_joint) const{
    switch (actuated_joint){
        case 0 :
        case 4 :
            return m_trajectory_adpf;
        
        case 1 :
        case 5 :
            return m_trajectory_haa;
        
        case 2 :
        case 6 :
            return m_trajectory_hfe;
        
        case 3 :
        case 7 :
            return m_trajectory_kfe;

        default :
            return std::vector<double>();
    }
}

void TestJointsGaitPlanning::setGaitType(const exoMode &new_gait_type){
    m_gait_type = new_gait_type; 
}

exoMode TestJointsGaitPlanning::getGaitType() const{
    return m_gait_type; 
}

void TestJointsGaitPlanning::setPrevPoint(const std::vector<double> &point){
    m_prev_point = point; 
}

std::vector<double> TestJointsGaitPlanning::getPrevPoint() const{
    return m_prev_point; 
}