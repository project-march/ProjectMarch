/*Authors: Femke Buiks and Andrew Hutani, MIX*/

#pragma once

#include <string>
#include <vector>
#include <array>  
#include "../../state_machine/include/state_machine/exo_state.hpp"

class GaitPlanning {
    
public:
    explicit GaitPlanning();

    // Setters
    void setStanceFoot(const int &new_stance_foot);
    void setFootPositions(const std::array<double, 3> &new_left_foot_position, const std::array<double, 3> &new_right_foot_position);
    void setGaitType(const exoState &new_gait_type);  
    void setBezierGait();

    // Getters
    std::vector<std::array<double, 4>> getTrajectory() const;
    int getCurrentStanceFoot() const; 
    std::array<double, 3> getCurrentLeftFootPos() const; 
    std::array<double, 3> getCurrentRightFootPos() const; 
    exoState getGaitType() const; 

private: 
    exoState m_gait_type; 
    int m_current_stance_foot; 
    double m_step_size; 
    std::array<double, 3> m_current_left_foot_position; 
    std::array<double, 3> m_current_right_foot_position; 
    std::vector<std::array<double, 4>> m_bezier_trajectory; 
    std::vector<std::array<double, 4>> m_first_step_trajectory; 

    std::vector<std::array<double, 4>> processCSV(const std::string& filename);
    
}; 