#include "com_estimator.hpp"
#include "state_estimator.hpp"

ComEstimator::ComEstimator(StateEstimator* owner)
    : m_owner(owner)
    , m_com_position(geometry_msgs::msg::Point())
{
}

geometry_msgs::msg::PointStamped ComEstimator::get_com_state()
{
    geometry_msgs::msg::PointStamped com_state;
    com_state.header.stamp = m_owner->get_clock()->now();
    com_state.point = m_com_position;
    return com_state;
}

void ComEstimator::set_com_state(std::vector<geometry_msgs::msg::TransformStamped> transforms)
{
}