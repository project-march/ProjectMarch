#include "march_gait_planning/gait_planning_base_class.hpp"

GaitPlanning::GaitPlanning()
 : m_gait_type(), 
   m_previous_gait_type()
{
    std::cout << "Gait Planning base class created" << std::endl; 
}

void GaitPlanning::setGaitType(const exoMode &new_gait_type){
    m_gait_type = new_gait_type; 
}

void GaitPlanning::setPreviousGaitType(const exoMode &previous_gait_type){
    m_previous_gait_type = previous_gait_type; 
}

exoMode GaitPlanning::getGaitType() const {
    return m_gait_type; 
}

exoMode GaitPlanning::getPreviousGaitType() const {
    return m_previous_gait_type; 
}
