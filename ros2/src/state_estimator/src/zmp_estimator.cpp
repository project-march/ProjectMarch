#include "state_estimator/zmp_estimator.hpp"

ZmpEstimator::ZmpEstimator()
{
}

void ZmpEstimator::set_zmp(CenterOfMass com, CenterOfPressure cop)
{

}

geometry_msgs::msg::PointStamped ZmpEstimator::get_zmp()
{
    return m_position;
}