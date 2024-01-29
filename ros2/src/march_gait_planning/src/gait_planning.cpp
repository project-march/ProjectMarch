/*Authors: Femke Buiks and Andrew Hutani, MIX*/

#include "march_gait_planning/gait_planning.hpp"
#include <tuple>
#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream>

#include "ament_index_cpp/get_package_share_directory.hpp"

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
  m_step_size(), 
  m_current_left_foot_position(), 
  m_current_right_foot_position(), 
  m_large_bezier_trajectory(),
  m_large_first_step_trajectory(),
  m_small_bezier_trajectory(), 
  m_small_first_step_trajectory() 
  {
    std::cout << "Gait Planning Class created" << std::endl; 
    setBezierGait(); 
    std::cout << "Bezier CSV created" << std::endl; 
  }

void GaitPlanning::setStanceFoot(const uint8_t &new_stance_foot){
    m_current_stance_foot = new_stance_foot; 
}

void GaitPlanning::setFootPositions(const XYZFootPositionArray &new_left_foot_position, const XYZFootPositionArray &new_right_foot_position) { 
    m_current_left_foot_position = new_left_foot_position; 
    m_current_right_foot_position = new_right_foot_position; 
}

void GaitPlanning::setGaitType(const exoMode &new_gait_type){
    m_gait_type = new_gait_type; 
}

void GaitPlanning::setBezierGait(){
    std::string cartesian_files_directory = ament_index_cpp::get_package_share_directory("march_gait_planning") + "/m9_gait_files/cartesian/";
    m_large_first_step_trajectory = processCSV(cartesian_files_directory + "first_step_large.csv");
    m_large_bezier_trajectory = processCSV(cartesian_files_directory + "normal_gait_large.csv");
    m_small_first_step_trajectory = processCSV(cartesian_files_directory + "first_step_small.csv");
    m_small_bezier_trajectory = processCSV(cartesian_files_directory + "normal_gait_small.csv");
}

std::vector<XZFeetPositionsArray> GaitPlanning::getTrajectory() const{
    std::vector<XZFeetPositionsArray> result;  
    switch (m_gait_type){
        case exoMode::LargeWalk : 
        return (m_current_stance_foot & 0b11) ? m_large_first_step_trajectory : m_large_bezier_trajectory; 
        case exoMode::SmallWalk : 
        return  (m_current_stance_foot & 0b11) ? m_small_first_step_trajectory : m_small_bezier_trajectory; 
        default : 
        return {}; 
    }
    // return result; 
}

int GaitPlanning::getCurrentStanceFoot() const {
    return m_current_stance_foot; 
}

XYZFootPositionArray GaitPlanning::getCurrentLeftFootPos() const{
    return m_current_left_foot_position; 
}

XYZFootPositionArray GaitPlanning::getCurrentRightFootPos() const{
    return m_current_right_foot_position; 
}

exoMode GaitPlanning::getGaitType() const{
    return m_gait_type; 
}

// This getter can also be included in the general getTrajectory function, depending on how we identify
// the camera's as being used for input. 
std::vector<XZFeetPositionsArray> GaitPlanning::getVariableTrajectory() const{
    return m_variable_step_trajectory; 
}

std::vector<double> GaitPlanning::linspace(const double &min, const double &max, const int &size)
{
	std::vector<double> result;
	int iterator = 0;
	for (int i = 0; i <= size-2; i++)	
	{
		double temp = min + i*(max-min)/(floor((double)size) - 1);
		result.insert(result.begin() + iterator, temp);
		iterator += 1;
	}
	result.insert(result.begin() + iterator, max);
	return result;
}

std::vector<XZFeetPositionsArray> GaitPlanning::interpolateVariableTrajectory(const float &step_distance){
    m_variable_step_trajectory.clear(); 
    int length = std::end(m_small_first_step_trajectory)-std::begin(m_small_first_step_trajectory); 
    std::vector<double> x_right= linspace(0, step_distance/2, length);  
    std::vector<double> x_left = linspace(0, -step_distance/2, length); 
    std::vector<double> z_left(length, 0.0); 
    std::vector<XZFeetPositionsArray> finish_step;
    for (int i=0; i < length; i++){
        float z= m_small_first_step_trajectory[i][1] + (x_right[i] - m_small_first_step_trajectory[i][0])*((m_large_first_step_trajectory[i][1]-m_small_first_step_trajectory[i][1])/(m_large_first_step_trajectory[i][0]-m_small_first_step_trajectory[i][0])); 
        if (z != z){
            m_variable_step_trajectory.push_back({x_right[i], 0.0, x_left[i], z_left[i]}); 
        } else {
        m_variable_step_trajectory.push_back({x_right[i], z, x_left[i], z_left[i]});       
        }
    }
    for (int k = 0; k < length; k++){
        float z= m_small_first_step_trajectory[k][1] + (x_right[k] - m_small_first_step_trajectory[k][0])*((m_large_first_step_trajectory[k][1]-m_small_first_step_trajectory[k][1])/(m_large_first_step_trajectory[k][0]-m_small_first_step_trajectory[k][0]));
        if (z != z){
            finish_step.push_back({x_left[k]+(step_distance/2), z_left[k], x_right[k]-(step_distance/2), 0.0});        
        } else {
        finish_step.push_back({x_left[k]+(step_distance/2), z_left[k], x_right[k]-(step_distance/2), z}); 
        }    
    }
    m_variable_step_trajectory.insert(m_variable_step_trajectory.end(), finish_step.begin(), finish_step.end()); 
    return m_variable_step_trajectory;  
}

std::vector<XZFeetPositionsArray> GaitPlanning::processCSV(const std::string& filename){
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

    std::vector<XZFeetPositionsArray> trajectory;
    for (const auto& row : data) {
        trajectory.push_back({std::stod(row.x_swing), std::stod(row.z_swing), std::stod(row.x_stance), std::stod(row.z_stance)}); 
    }

    return trajectory;
}