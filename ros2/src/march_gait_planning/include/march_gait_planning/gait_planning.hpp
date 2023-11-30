#pragma once

#include <string>
#include <vector>
#include <array>  
#include "../../state_machine/include/state_machine/exo_state.hpp"

class GaitPlanning {
    
    public:
    explicit GaitPlanning(); 
    std::vector<double> getFootEndPositions() const; 
    double getStepSize() const; 
    void createBezierTrajectory(); 
    std::array<double, 3> timeBufferBezier(); //TODO: put in node class 
    void setStanceFoot(const int &new_stance_foot);
    void setFootPositions(const std::array<double, 3> &new_left_foot_position, const std::array<double, 3> &new_right_foot_position); // set foot positions (l+r) and current stance leg 
    void setGaitType(const exoState &new_gait_type); 
    void setPathToYaml(const std::string &path_to_yaml); //TODO: create yaml in gait_planning

    //ADD STANCE FOOT COORDINATES AS OUTPUT 

    private: 
    exoState m_gait_type; 
    int m_current_stance_foot; 
    std::array<double, 3> m_current_left_foot_position; 
    std::array<double, 3> m_current_right_foot_position; 
    std::string m_path_to_yaml; 
    std::vector<std::array<double, 3>> m_bezier_trajectory; 
    
}; 