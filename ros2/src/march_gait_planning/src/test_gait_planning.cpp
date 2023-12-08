//TODO: Fix the header files
#include "march_gait_planning/test_gait_planning.hpp"
#include <tuple>
#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream>

struct CSVRow {
    double angle;
};

TestGaitPlanning::TestGaitPlanning()
: m_gait_type(), 
  m_trajectory()
  {
    std::cout << "Test Gait Planning Class created" << std::endl; 
    setTrajectory(); 
    std::cout << "Bezier CSV created" << std::endl; 
  }

void TestGaitPlanning::setTrajectory(){
    std::vector<CSVRow> data;
    
    //TODO: Change the files for the gaits
    std::ifstream file("src/march_gait_planning/m9_gait_files/test_trajectory.csv");

    if (!file.is_open()) {
        std::cerr << "Error opening file." << std::endl;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        double angle;
        if (!(iss >> angle)) { break; }  // error
        // push angle to the vector.
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
