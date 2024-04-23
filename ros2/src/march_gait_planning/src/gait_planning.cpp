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
  m_previous_gait_type(),
  m_current_stance_foot(), 
  m_step_size(), 
  m_current_left_foot_position(), 
  m_current_right_foot_position(), 
  m_large_bezier_trajectory(),
  m_large_first_step_trajectory(),
  m_small_bezier_trajectory(), 
  m_small_first_step_trajectory(),
  m_large_step_close_trajectory(),
  m_small_step_close_trajectory(),
  m_high_step_26cm_trajectory(),
  m_ascending_trajectory()
  {
    std::cout << "Gait Planning Class created" << std::endl; 
    setBezierGait(); 
    std::cout << "Bezier CSV created" << std::endl; 
  }

void GaitPlanning::setStanceFoot(const uint8_t &new_stance_foot){
    m_current_stance_foot = new_stance_foot; 
}

void GaitPlanning::setFootPositions(const GaitPlanning::XYZFootPositionArray &new_left_foot_position, const GaitPlanning::XYZFootPositionArray &new_right_foot_position) { 
    m_current_left_foot_position = new_left_foot_position; 
    m_current_right_foot_position = new_right_foot_position; 
}

void GaitPlanning::setGaitType(const exoMode &new_gait_type){
    m_gait_type = new_gait_type; 
}

void GaitPlanning::setPreviousGaitType(const exoMode &previous_gait_type){
    m_previous_gait_type = previous_gait_type; 
}

void GaitPlanning::setBezierGait(){
    std::string cartesian_files_directory = ament_index_cpp::get_package_share_directory("march_gait_planning") + "/m9_gait_files/cartesian/";
    m_large_first_step_trajectory = processCSV(cartesian_files_directory + "first_step_large.csv");
    m_large_bezier_trajectory = processCSV(cartesian_files_directory + "normal_gait_large.csv");
    m_small_first_step_trajectory = processCSV(cartesian_files_directory + "first_step_small.csv");
    m_small_bezier_trajectory = processCSV(cartesian_files_directory + "normal_gait_small.csv");
    m_large_step_close_trajectory = processCSV(cartesian_files_directory + "large_step_close.csv");
    m_small_step_close_trajectory = processCSV(cartesian_files_directory + "small_step_close.csv");
    m_high_step_26cm_trajectory = processCSV(cartesian_files_directory + "high_step1.csv");
    m_high_step_26cm_close_trajectory = processCSV(cartesian_files_directory + "high_step1_down.csv"); 
    m_high_step_22cm_trajectory = processCSV(cartesian_files_directory + "high_step2.csv");
    m_high_step_22cm_close_trajectory = processCSV(cartesian_files_directory + "high_step2_down.csv"); 
    m_high_step_18cm_trajectory = processCSV(cartesian_files_directory + "high_step3.csv");
    m_high_step_18cm_close_trajectory = processCSV(cartesian_files_directory + "high_step3_down.csv"); 
    m_ascending_trajectory = processCSV(cartesian_files_directory + "ascend_test.csv");
    m_descending_trajectory = processCSV(cartesian_files_directory + "descend_test.csv"); 
}

void GaitPlanning::setVariableDistance(const float &distance){
    m_variable_distance = distance; 
}

std::vector<GaitPlanning::XZFeetPositionsArray> GaitPlanning::getTrajectory() {
    // std::vector<GaitPlanning::XZFeetPositionsArray> result;  
    switch (m_gait_type){
        case exoMode::LargeWalk : 
            return (m_current_stance_foot == 3) ? m_large_first_step_trajectory : m_large_bezier_trajectory; 
        case exoMode::SmallWalk : 
            return  (m_current_stance_foot == 3) ? m_small_first_step_trajectory : m_small_bezier_trajectory; 
        case exoMode::Stand :
            switch (m_previous_gait_type){
                case exoMode::LargeWalk :
                    return m_large_step_close_trajectory; 
                case exoMode::SmallWalk :
                    return m_small_step_close_trajectory; 
                case exoMode::HighStep1 : 
                    return m_high_step_26cm_close_trajectory; 
                case exoMode::HighStep2 : 
                    return m_high_step_22cm_close_trajectory; 
                case exoMode::HighStep3 : 
                    return m_high_step_18cm_close_trajectory; 
                case exoMode::VariableWalk : 
                    return variableFirstStepTrajectory(m_variable_distance); 
                default :
                    return {}; 
            }
            // return (m_previous_gait_type == exoMode::LargeWalk) ? m_large_step_close_trajectory : m_small_step_close_trajectory;
        case exoMode::HighStep1 :
            return m_high_step_26cm_trajectory;
        case exoMode::HighStep2 :
            return m_high_step_22cm_trajectory;
        case exoMode::HighStep3 :
            return m_high_step_18cm_trajectory;
            return (m_previous_gait_type == exoMode::LargeWalk) ? m_large_step_close_trajectory : m_small_step_close_trajectory;
        case exoMode::Ascending :
            return m_ascending_trajectory;
        case exoMode::Descending :
            return m_descending_trajectory; 
        default : 
            return {}; 
    }
}

int GaitPlanning::getCurrentStanceFoot() const {
    return m_current_stance_foot; 
}

GaitPlanning::XYZFootPositionArray GaitPlanning::getCurrentLeftFootPos() const{
    return m_current_left_foot_position; 
}

GaitPlanning::XYZFootPositionArray GaitPlanning::getCurrentRightFootPos() const{
    return m_current_right_foot_position; 
}

exoMode GaitPlanning::getGaitType() const{
    return m_gait_type; 
}

exoMode GaitPlanning::getPreviousGaitType() const{
    return m_previous_gait_type; 
}

float GaitPlanning::getVariableDistance() const{
    return m_variable_distance; 
}

// This getter can also be included in the general getTrajectory function, depending on how we identify
// the camera's as being used for input. 
std::vector<GaitPlanning::XZFeetPositionsArray> GaitPlanning::getVariableTrajectory() const{
    return m_variable_step_trajectory; 
}

std::vector<GaitPlanning::XZFeetPositionsArray> GaitPlanning::variableStepStoneTrajectory(const float &step_distance) {

    m_variable_step_trajectory.clear(); 
    std::vector<GaitPlanning::XZFeetPositionsArray> right_open = interpolateVariableTrajectory(step_distance, m_small_first_step_trajectory, m_large_first_step_trajectory, false); 
    m_variable_step_trajectory.insert(m_variable_step_trajectory.end(), right_open.begin(), right_open.end());
    std::vector<GaitPlanning::XZFeetPositionsArray> left_close = interpolateVariableTrajectory(step_distance, m_small_first_step_trajectory, m_large_first_step_trajectory, false);
    
    // We need to swap the columns as right foot and left foot need to be switched (left foot is now the swing foot that performs the closing step), and the interpolateVariableTrajectory method always places right foot first. An offset also needs to be added as the feet will begin in opposite direction. Now left is swing and right is stance. NOTE: if the exo is positioned opposite, the trajectory needs to be swapped again in the node message. 
    for (int k = 0; k < static_cast<int>(left_close.size()); k++){
        left_close[k][0] -= (step_distance/2); 
        left_close[k][2] += (step_distance/2); 
        std::swap(left_close[k][0], left_close[k][2]); 
        std::swap(left_close[k][1], left_close[k][3]);
    }

    m_variable_step_trajectory.insert(m_variable_step_trajectory.end(), left_close.begin(), left_close.end());
    return m_variable_step_trajectory; 

}

float GaitPlanning::interpolateZ(float x1, float z1, float x2, float z2, float x) {
    return z1 + (x - x1) * ((z2 - z1) / (x2 - x1));
}

std::vector<GaitPlanning::XZFeetPositionsArray> GaitPlanning::variableFirstStepTrajectory(const float &step_distance){
    m_variable_step_trajectory.clear(); 
    m_variable_step_trajectory = interpolateVariableTrajectory(step_distance, m_small_first_step_trajectory, m_large_first_step_trajectory, false); 
    return m_variable_step_trajectory; 

}

std::vector<GaitPlanning::XZFeetPositionsArray> GaitPlanning::variableFullStepTrajectory(const float &step_distance){
    m_variable_step_trajectory.clear(); 
    m_variable_step_trajectory = interpolateVariableTrajectory(step_distance, m_small_bezier_trajectory, m_large_bezier_trajectory, true); 
    return m_variable_step_trajectory; 
}

std::vector<GaitPlanning::XZFeetPositionsArray> GaitPlanning::variableCloseStepTrajectory(const float &step_distance){

    // //Maybe this can just be the variableFirstStepTrajectory but published on alternate feet in the node??? Aka remove this method entirely and just change the order of publishing. 
   
    // m_variable_step_trajectory.clear();

    // // Calculate array length
    // int array_length = m_small_first_step_trajectory.size();

    // // Interpolate the LEFT swing step close
    // for (int k = 0; k < array_length; k++) {
    //     float x_right_k = step_distance / 2 * static_cast<float>(k) / static_cast<float>(array_length - 1);
    //     float x_left_k = -step_distance / 2 * static_cast<float>(k) / static_cast<float>(array_length - 1);
    //     float z = interpolateZ(m_small_first_step_trajectory[k][0], m_small_first_step_trajectory[k][1],
    //                            m_large_first_step_trajectory[k][0], m_large_first_step_trajectory[k][1], x_right_k);
    //     if (std::isnan(z)) {
    //         m_variable_step_trajectory.push_back({x_left_k + step_distance / 2, 0.0, x_right_k - step_distance / 2, 0.0});
    //     } else {
    //         m_variable_step_trajectory.push_back({x_left_k + step_distance / 2, 0.0, x_right_k - step_distance / 2, z});
    //     }
    // }

    // return m_variable_step_trajectory;

    m_variable_step_trajectory.clear(); 
    m_variable_step_trajectory = interpolateVariableTrajectory(step_distance, m_small_first_step_trajectory, m_large_first_step_trajectory, false); 
    for (int k = 0; k < static_cast<int>(m_variable_step_trajectory.size()); k++){
        m_variable_step_trajectory[k][0] -= (step_distance/2); 
        m_variable_step_trajectory[k][2] += (step_distance/2); 
        // std::swap(m_variable_step_trajectory[k][0], m_variable_step_trajectory[k][2]); 
        // std::swap(m_variable_step_trajectory[k][1], m_variable_step_trajectory[k][3]);
    }
    return m_variable_step_trajectory; 

}

std::vector<GaitPlanning::XZFeetPositionsArray> GaitPlanning::interpolateVariableTrajectory(const float &step_distance, const std::vector<GaitPlanning::XZFeetPositionsArray> &small_trajectory, const std::vector<GaitPlanning::XZFeetPositionsArray> &large_trajectory, bool full_step){

    std::vector<GaitPlanning::XZFeetPositionsArray> temp_vector; 
    
    int array_length = small_trajectory.size();
    float factor = full_step ? 1.0f : 0.5f; 

    for (int i = 0; i < array_length; i++){
        float x_right_i = step_distance * factor * static_cast<float>(i) / static_cast<float>(array_length - 1);
        float x_left_i = -step_distance * factor * static_cast<float>(i) / static_cast<float>(array_length - 1);
        float z = interpolateZ(small_trajectory[i][0], small_trajectory[i][1],
                               large_trajectory[i][0], large_trajectory[i][1], x_right_i);
        if (std::isnan(z)) {
            temp_vector.push_back({x_right_i, 0.0, x_left_i, 0.0});
        } else {
            temp_vector.push_back({x_right_i, z, x_left_i, 0.0});
        }
    }

    return temp_vector; 
}

std::vector<GaitPlanning::XZFeetPositionsArray> GaitPlanning::processCSV(const std::string& filename){
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

    std::vector<GaitPlanning::XZFeetPositionsArray> trajectory;
    for (const auto& row : data) {
        trajectory.push_back({std::stod(row.x_swing), std::stod(row.z_swing), std::stod(row.x_stance), std::stod(row.z_stance)}); 
    }

    return trajectory;
}