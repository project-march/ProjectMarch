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
CopEstimator::CopEstimator(std::vector<PressureSensor*>* sensors)
    : m_sensors(sensors)
{
}
/**
 * This function updates the cop of the exo be recalculating the cop with new input data.
 * Here the average x, y and z coordinates are calculated individually.
 * If no pressure is measured on all sensors, an error is thrown.
 * @param sensors
 */
void CopEstimator::set_cop_state(
    std::vector<PressureSensor> sensors, std::array<geometry_msgs::msg::TransformStamped, 2> reference_frames)
{
    m_center_of_pressure.pressure = 0;
    m_center_of_pressure.position.point.x = 0;
    m_center_of_pressure.position.point.y = 0;
    m_center_of_pressure.position.point.z = 0;

    geometry_msgs::msg::PointStamped transformed_point;

    for (const auto& i : sensors) {
        auto pressure = i.centre_of_pressure.pressure;
        m_center_of_pressure.pressure += pressure;

        tf2::doTransform(i.centre_of_pressure.position, transformed_point, reference_frames[(i.name[0] == *"l")]);

        m_center_of_pressure.position.point.x += transformed_point.point.x * pressure;
        m_center_of_pressure.position.point.y += transformed_point.point.y * pressure;
        m_center_of_pressure.position.point.z += transformed_point.point.z * pressure;
    }
    if (m_center_of_pressure.pressure != 0) {
        m_center_of_pressure.position.point.x = m_center_of_pressure.position.point.x / m_center_of_pressure.pressure;
        m_center_of_pressure.position.point.y = m_center_of_pressure.position.point.y / m_center_of_pressure.pressure;
        m_center_of_pressure.position.point.z = m_center_of_pressure.position.point.z / m_center_of_pressure.pressure;
    } else {
        RCLCPP_WARN(rclcpp::get_logger("cop_estimator"), "All pressure sensors have pressure 0.");
        //        throw std::runtime_error("ERROR: The total measured pressure is 0.\n");
    }
    RCLCPP_DEBUG(
        rclcpp::get_logger("cop_estimator"), "All pressure sensors have pressure of %f", m_center_of_pressure.pressure);
}

void CopEstimator::update_sensor_pressures(std::map<std::string, double> pressure_values_map)
{
    for (auto it = pressure_values_map.begin(); it != pressure_values_map.end(); ++it) {
        update_individual_pressure_sensor(it->first, it->second);
    }
    // set_cop_state(m_sensors);
}

void CopEstimator::update_individual_pressure_sensor(std::string name, double pressure)
{
    auto it = std::find_if(m_sensors->begin(), m_sensors->end(), [name](const PressureSensor* sensor) {
        return sensor->name == name;
    });
    it->update_pressure(pressure);
}

/**
 * Returns the last calculated cop of the cop estimated
 * @return
 */
CenterOfPressure CopEstimator::get_cop_state()
{
    return m_center_of_pressure;
}

std::vector<PressureSensor*>* CopEstimator::get_sensors()
{
    return m_sensors;
}
