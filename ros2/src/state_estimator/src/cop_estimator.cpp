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
CopEstimator::CopEstimator(std::vector<PressureSensor*> sensors)
    : m_sensors(sensors)
{
}
/**
 * This function updates the cop of the exo be recalculating the cop with new input data.
 * Here the average x, y and z coordinates are calculated individually.
 * If no pressure is measured on all sensors, an error is thrown.
 * @param sensors
 */
void CopEstimator::set_cop(
    std::vector<PressureSensor*> sensors, std::array<geometry_msgs::msg::TransformStamped, 2> reference_frames)
{
    double total_pressure = 0.0;
    m_center_of_pressure.point.x = 0.0;
    m_center_of_pressure.point.y = 0.0;
    m_center_of_pressure.point.z = 0.0;

    geometry_msgs::msg::PointStamped transformed_point;

    for (const auto sensor : sensors) {
        auto pressure = sensor->pressure;
        total_pressure += pressure;

        tf2::doTransform(sensor->position, transformed_point, reference_frames[(sensor->name[0] == *"l")]);

        m_center_of_pressure.point.x += transformed_point.point.x * pressure;
        m_center_of_pressure.point.y += transformed_point.point.y * pressure;
        m_center_of_pressure.point.z += transformed_point.point.z * pressure;
    }
    if (total_pressure != 0) {
        m_center_of_pressure.point.x = m_center_of_pressure.point.x / total_pressure;
        m_center_of_pressure.point.y = m_center_of_pressure.point.y / total_pressure;
        m_center_of_pressure.point.z = m_center_of_pressure.point.z / total_pressure;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("cop_estimator"), "All pressure sensors have pressure 0.");
    }
}

/**
 * Updates the values of the pressure sensors map, matches name to pressure value.
 * Make use of a function that sets the individual sensor to the correct value.
 * @param pressure_values_map
 */
void CopEstimator::update_pressure_sensors(std::map<std::string, double> pressure_values_map)
{
    for (auto& x : pressure_values_map) {
        update_individual_pressure_sensor(x.first, x.second);
    }
}

/**
 * Sets the new pressure vlue of an individual pressure sensor.
 * It looks for the name in the sensor array and updates the pressure value of the sensor
 * @param name The name of the sensor to update
 * @param pressure The new pressure value.
 */
void CopEstimator::update_individual_pressure_sensor(std::string name, double pressure)
{
    for (auto x : m_sensors) {
        if (x->name == name) {
            x->pressure = pressure;
            return;
        }
    }
}

/**
 * Returns the last calculated cop of the cop estimated
 * @return
 */
geometry_msgs::msg::PointStamped CopEstimator::get_cop()
{
    return m_center_of_pressure;
}

/**
 * Returns the pressure sensors.
 * @return
 */
std::vector<PressureSensor*>* CopEstimator::get_sensors()
{
    return &m_sensors;
}
