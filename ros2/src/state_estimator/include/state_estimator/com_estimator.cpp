#include "com_estimator.hpp"
#include "state_estimator.hpp"

ComEstimator::ComEstimator(StateEstimator* owner)
    : m_owner(owner)
{
}

void ComEstimator::set_com_state(std::vector<CenterOfMass> mass_list)
{
    m_center_of_mass.mass = 0;
    m_center_of_mass.position.point.x = 0;
    m_center_of_mass.position.point.y = 0;
    m_center_of_mass.position.point.z = 0;
    for (auto i : mass_list) {
        m_center_of_mass.mass += i.mass;
        m_center_of_mass.position.point.x += i.position.point.x * i.mass;
        m_center_of_mass.position.point.y += i.position.point.y * i.mass;
        m_center_of_mass.position.point.z += i.position.point.z * i.mass;
    }

    m_center_of_mass.position.point.x = m_center_of_mass.position.point.x / m_center_of_mass.mass;
    m_center_of_mass.position.point.y = m_center_of_mass.position.point.y / m_center_of_mass.mass;
    m_center_of_mass.position.point.z = m_center_of_mass.position.point.z / m_center_of_mass.mass;
}

CenterOfMass ComEstimator::get_com_state()
{
    return m_center_of_mass;
}