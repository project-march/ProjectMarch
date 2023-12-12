#include "march_gait_planning/gait_planning_joint_angles.hpp"

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

GaitPlanningAngles::GaitPlanningAngles()
 : m_gait_type(), 
   m_angle_trajectory(), 
   m_prev_point(), 
   m_counter()
   {
    std::cout << "Angle Gait Class created" << std::endl; 
    setAngleCSV(); 
    std::cout << "Angle trajectory CSV created" << std::endl; 
   }

void GaitPlanningAngles::setAngleCSV(){
    std::vector<CSVRow> data; 
    std::ifstream file("src/march_gait_planning/m9_gait_files/q_first_steps.csv"); 
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
        m_angle_trajectory.push_back({std::stod(row.left_hip_aa), std::stod(row.left_hip_fe), std::stod(row.left_knee), std::stod(row.left_ankle), std::stod(row.right_hip_aa), std::stod(row.right_hip_fe), std::stod(row.right_knee), std::stod(row.right_ankle)}); 
    }

    m_counter = 0; 
    m_prev_point = m_angle_trajectory[0]; 
}

void GaitPlanningAngles::setGaitType(const exoState &new_gait_type){
    m_gait_type = new_gait_type; 
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

std::vector<double> GaitPlanningAngles::getPrevPoint() const{
    return m_prev_point; 
}

int GaitPlanningAngles::getCounter() const{
    return m_counter; 
}

std::vector<std::vector<double>> GaitPlanningAngles::getAngleTrajectory() const{
    return m_angle_trajectory; 
}


