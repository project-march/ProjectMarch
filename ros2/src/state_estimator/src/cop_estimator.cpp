//
// Created by marco on 14-2-23.
//
#include "state_estimator/cop_estimator.hpp"
#include "state_estimator/state_estimator.hpp"
/**
 * Creates a CopEstimator object.
 *
 * The Center of pressure estimator is used to calculate the center of pressure of the exo.
 * For this calculation the input of the pressure soles is used.
 * @param sensors
 */
CopEstimator::CopEstimator(std::vector<PressureSensor> sensors)
    : m_sensors(sensors)
{
    set_cop_state(sensors);
}
/**
 * This function updates the cop of the exo be recalculating the cop with new input data.
 * Here the average x, y and z coordinates are calculated individually.
 * If no pressure is measured on all sensors, an error is thrown.
 * @param sensors
 */
void CopEstimator::set_cop_state(std::vector<PressureSensor> sensors)
{
    m_center_of_pressure.pressure = 0;
    m_center_of_pressure.position.point.x = 0;
    m_center_of_pressure.position.point.y = 0;
    m_center_of_pressure.position.point.z = 0;
    for (const auto& i : sensors) {
        auto pressure = i.centre_of_pressure.pressure;
        m_center_of_pressure.pressure += pressure;
        m_center_of_pressure.position.point.x += i.centre_of_pressure.position.point.x * pressure;
        m_center_of_pressure.position.point.y += i.centre_of_pressure.position.point.y * pressure;
        m_center_of_pressure.position.point.z += i.centre_of_pressure.position.point.z * pressure;
    }
    if (m_center_of_pressure.pressure != 0) {
        m_center_of_pressure.position.point.x = m_center_of_pressure.position.point.x / m_center_of_pressure.pressure;
        m_center_of_pressure.position.point.y = m_center_of_pressure.position.point.y / m_center_of_pressure.pressure;
        m_center_of_pressure.position.point.z = m_center_of_pressure.position.point.z / m_center_of_pressure.pressure;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("cop_estimator"), "All pressure sensors have pressure 0.");
        throw std::runtime_error("ERROR: The total measured pressure is 0.\n");
    }
}

void CopEstimator::update_sensor_pressures(std::map<std::string, double> pressure_values_map)
{
    for (auto& sensor : m_sensors) {
        sensor.update_pressure(pressure_values_map.at(sensor.name));
    }
}

/**
 * Returns the last calculated cop of the cop estimated
 * @return
 */
CenterOfPressure CopEstimator::get_cop_state()
{
    return m_center_of_pressure;
}
