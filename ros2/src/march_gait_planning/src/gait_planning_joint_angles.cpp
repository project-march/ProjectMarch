/*
Authors: Femke Buiks, MIX

This file contains the functionality of the GaitPlanningAngles class.  This class allows the definition of a gait in joint angles. 

*/ 

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
   m_complete_step_angle_trajectory(),
   m_half_step_angle_trajectory(),
   m_stand_to_sit_trajectory(), 
   m_sideways_right_trajectory(),
   m_sideways_left_trajectory(),
   m_sit_to_stand_trajectory(),
   m_step_close_trajectory(), 
   m_home_stand(),
   m_prev_point(), 
   m_hip_tilt(), 
   m_counter()
   {
    std::cout << "Angle Gait Class created" << std::endl;
    processCSVFile("src/march_gait_planning/m9_gait_files/joint_angles/cartesian_in_joint_angle_first_step.csv", m_first_step_angle_trajectory); 
    processCSVFile("src/march_gait_planning/m9_gait_files/joint_angles/cartesian_in_joint_angle_full_step.csv", m_complete_step_angle_trajectory);
    processCSVFile("src/march_gait_planning/m9_gait_files/joint_angles/cartesian_in_joint_angle_step_close.csv", m_step_close_trajectory); 
    processCSVFile("src/march_gait_planning/m9_gait_files/joint_angles/cartesian_in_joint_angle_half_step.csv", m_half_step_angle_trajectory); 

    processCSVFile("src/march_gait_planning/m9_gait_files/joint_angles/stand_to_sit.csv", m_stand_to_sit_trajectory); 
    processCSVFile("src/march_gait_planning/m9_gait_files/joint_angles/sidestep_right.csv", m_sideways_right_trajectory); 
    processCSVFile("src/march_gait_planning/m9_gait_files/joint_angles/sidestep_left.csv", m_sideways_left_trajectory);

    processCSVFile("src/march_gait_planning/m9_gait_files/joint_angles/sit_to_stand.csv", m_sit_to_stand_trajectory); 
    processCSVFile("src/march_gait_planning/m9_gait_files/joint_angles/hinge_gait.csv", m_hinge_trajectory);

    processCSVFile("src/march_gait_planning/m9_gait_files/joint_angles/stand_to_trainsit.csv", m_stand_to_train_sit_trajectory);
    processCSVFile("src/march_gait_planning/m9_gait_files/joint_angles/trainsit_to_stand.csv", m_train_sit_to_stand_trajectory);

    std::cout << "Angle trajectory CSVs created" << std::endl; 
   }

void GaitPlanningAngles::processCSVFile(const std::string &path, std::vector<std::vector<double>> &member_variable){
    std::vector<CSVRow> data; 
    std::ifstream file(path); 
    if (!file.is_open()){
        std::cerr << "Error opening file" << path << std::endl; 
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

void GaitPlanningAngles::setGaitType(const ExoMode &new_gait_type){
    m_gait_type = new_gait_type; 
}

void GaitPlanningAngles::setPrevGaitType(const ExoMode &prev_gait_type){
    m_prev_gait_type = prev_gait_type; 
}

void GaitPlanningAngles::setCounter(const int &count){
    m_counter = count; 
}

void GaitPlanningAngles::setPrevPoint(const std::vector<double> &point){
    m_prev_point = point; 
}

void GaitPlanningAngles::setHomeStand(const std::vector<double> &stand){
    m_home_stand = stand; 
}

void GaitPlanningAngles::setStanceFoot(const uint8_t &foot){
    m_stance_foot = foot; 
}

void GaitPlanningAngles::setHipTilt(const double &hip_tilt){
    m_hip_tilt = hip_tilt; 
}

ExoMode GaitPlanningAngles::getGaitType() const {
    return m_gait_type; 
}

ExoMode GaitPlanningAngles::getPrevGaitType() const {
    return m_prev_gait_type; 
}

std::vector<double> GaitPlanningAngles::getPrevPoint() const{
    return m_prev_point; 
}

int GaitPlanningAngles::getCounter() const{
    return m_counter; 
}

std::vector<double> GaitPlanningAngles::getHomeStand() const{
    return m_home_stand; 
}

std::vector<std::vector<double>> GaitPlanningAngles::getFirstStepAngleTrajectory() const{
    return m_first_step_angle_trajectory; 
}

std::vector<std::vector<double>> GaitPlanningAngles::getFullGaitAngleCSV() const{
    switch (m_stance_foot){
        case RIGHT_STANCE_LEG :
        // right stance foot
        return m_half_step_angle_trajectory; 
        case LEFT_STANCE_LEG : {
        // left stance foot 
        std::vector<std::vector<double>> swapped_half_step = swapLeftAndRightColumns(m_half_step_angle_trajectory); 
        return swapped_half_step; 
        }
        default : 
        return m_half_step_angle_trajectory; 
    }
}

std::vector<std::vector<double>> GaitPlanningAngles::getStandToSitGait() const{
    return m_stand_to_sit_trajectory; 
}

std::vector<std::vector<double>> GaitPlanningAngles::getSidewaysRightGait() const{
    return m_sideways_right_trajectory; 
}

std::vector<std::vector<double>> GaitPlanningAngles::getSidewaysLeftGait() const{
    return m_sideways_left_trajectory; 
}

std::vector<std::vector<double>> GaitPlanningAngles::getSitToStandGait() const{
    return m_sit_to_stand_trajectory; 
}

std::vector<std::vector<double>> GaitPlanningAngles::getStepCloseGait() const{
    switch (m_stance_foot){
        case RIGHT_STANCE_LEG :
        return m_step_close_trajectory;  
        case LEFT_STANCE_LEG : {
        std::vector<std::vector<double>> swapped_step_close = swapLeftAndRightColumns(m_step_close_trajectory); 
        return swapped_step_close; 
        }
        default : 
        return m_step_close_trajectory; 
    }
}

std::vector<std::vector<double>> GaitPlanningAngles::getHingeGait() const{
    return m_hinge_trajectory; 
}

std::vector<std::vector<double>> GaitPlanningAngles::getStandToTrainSitGait() const{
    return m_stand_to_train_sit_trajectory; 
}

std::vector<std::vector<double>> GaitPlanningAngles::getTrainSitToStandGait() const{
    return m_train_sit_to_stand_trajectory; 
}

uint8_t GaitPlanningAngles::getStanceFoot() const {
    return m_stance_foot; 
}

std::vector<std::vector<double>> GaitPlanningAngles::swapLeftAndRightColumns(const std::vector<std::vector<double>>& matrix) const{
    std::vector<std::vector<double>> half_step_swapped = matrix; 
    for (auto& row : half_step_swapped){
        std::swap(row[0], row[4]);
        std::swap(row[1], row[5]);
        std::swap(row[2], row[6]);
        std::swap(row[3], row[7]);
    }
    return half_step_swapped; 
}

double GaitPlanningAngles::getHipTilt() const {
    return m_hip_tilt; 
}