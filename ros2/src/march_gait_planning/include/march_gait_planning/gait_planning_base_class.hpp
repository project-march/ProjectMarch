#include <array>
#include <vector>
#include <iostream>
#include "../../march_mode_machine/include/march_mode_machine/exo_mode.hpp"

class GaitPlanning {
    public: 

    //node class still needs to be able to use these 
    void setGaitType(const exoMode &new_gait_type); 
    void setPreviousGaitType(const exoMode &previous_gait_type);  
    exoMode getGaitType() const; 
    exoMode getPreviousGaitType() const;
    
    private: 
    exoMode m_gait_type; 
    exoMode m_previous_gait_type;

}; 