#include "march_gait_planning/gait_planning_joint_angles.hpp"

// Struct to keep track of all columns in the angle trajectory .csv files 
struct CSVRow {
    std::string left_hip_aa;
    std::string left_hip_fe;
    std::string left_knee; 
    std::string left_ankle; 
    std::string right_hip_aa;
    std::string right_hip_fe;
    std::string right_knee; 
    std::string right_ankle;
};

// Constructor for the GaitPlanningAngles class (functionality of joint angle trajectory)
GaitPlanningAngles::GaitPlanningAngles()
 : m_gait_type(), 
   m_prev_gait_type(), 
   m_first_step_angle_trajectory(), 
   m_prev_point(), 
   m_counter(), 
   m_complete_step_angle_trajectory(), 
   m_sideways_trajectory()
   {
    std::cout << "Angle Gait Class created" << std::endl;
    processCSVFile("src/march_gait_planning/m9_gait_files/joint_angles/first_step_q.csv", m_first_step_angle_trajectory); 
    processCSVFile("src/march_gait_planning/m9_gait_files/joint_angles/one_step_q.csv", m_complete_step_angle_trajectory);
    processCSVFile("src/march_gait_planning/m9_gait_files/joint_angles/stand_to_sit.csv", m_stand_to_sit_trajectory); 

    processCSVFile("src/march_gait_planning/m9_gait_files/joint_angles/sideways_walking_gait_dataset.csv", m_sideways_trajectory); 
    std::cout << "Angle trajectory CSVs created" << std::endl; 
   }

void GaitPlanningAngles::processCSVFile(const std::string &path, std::vector<std::vector<double>> &member_variable){
    std::vector<CSVRow> data; 
    std::ifstream file(path); 
    if (!file.is_open()){
        std::cerr << "Error opening file" << std::endl; 
    }
    std::string line; 
    while (std::getline(file, line)){
        std::istringstream iss(line); 
        CSVRow row; 
        std::getline(iss, row.left_hip_aa, ','); 
        std::getline(iss, row.left_hip_fe, ',');
        std::getline(iss, row.left_knee, ',');
        std::getline(iss, row.left_ankle, ',');
        std::getline(iss, row.right_hip_aa, ',');
        std::getline(iss, row.right_hip_fe, ',');
        std::getline(iss, row.right_knee, ',');
        std::getline(iss, row.right_ankle, ','); 
        data.push_back(row); 
    }

    file.close(); 

    for (const auto& row : data){
        member_variable.push_back({std::stod(row.left_hip_aa), std::stod(row.left_hip_fe), std::stod(row.left_knee), std::stod(row.left_ankle), std::stod(row.right_hip_aa), std::stod(row.right_hip_fe), std::stod(row.right_knee), std::stod(row.right_ankle)}); 
    }

    m_counter = 0; 
    m_prev_point = member_variable[0];
}

void GaitPlanningAngles::setGaitType(const exoState &new_gait_type){
    m_gait_type = new_gait_type; 
}

void GaitPlanningAngles::setPrevGaitType(const exoState &prev_gait_type){
    m_prev_gait_type = prev_gait_type; 
}

void GaitPlanningAngles::setCounter(const int &count){
    m_counter = count; 
}

void GaitPlanningAngles::setPrevPoint(const std::vector<double> &point){
    m_prev_point = point; 
}

exoState GaitPlanningAngles::getGaitType() const {
    return m_gait_type; 
}

exoState GaitPlanningAngles::getPrevGaitType() const {
    return m_prev_gait_type; 
}

std::vector<double> GaitPlanningAngles::getPrevPoint() const{
    return m_prev_point; 
}

int GaitPlanningAngles::getCounter() const{
    return m_counter; 
}

std::vector<std::vector<double>> GaitPlanningAngles::getFirstStepAngleTrajectory() const{
    return m_first_step_angle_trajectory; 
}

std::vector<std::vector<double>> GaitPlanningAngles::getFullGaitAngleCSV() const{
    return m_complete_step_angle_trajectory; 
}

std::vector<std::vector<double>> GaitPlanningAngles::getStandToSitGait() const{
    return m_stand_to_sit_trajectory; 
}

std::vector<std::vector<double>> GaitPlanningAngles::getSidewaysGait() const{
    return m_sideways_trajectory; 
}




