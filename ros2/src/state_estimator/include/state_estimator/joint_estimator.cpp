#include "joint_estimator.hpp"

JointEstimator::JointEstimator(StateEstimator* owner, sensor_msgs::msg::JointState initial_joint_states)
    :m_owner(owner)
    ,m_joint_states(initial_joint_states)
{
    
}

void JointEstimator::set_joint_states(sensor_msgs::msg::JointState& new_joint_states)
{

}

sensor_msgs::msg::JointState JointEstimator::get_joint_states()
{
    return m_joint_states;
}