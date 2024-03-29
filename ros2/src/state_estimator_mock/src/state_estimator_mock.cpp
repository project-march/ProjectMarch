//
// Created by Jack Zeng march8 on 30-05-2023
//

#include "state_estimator_mock/state_estimator_mock.hpp"
#include "state_estimator_mock/state_estimator_mock_node.hpp"

StateEstimatorMock::StateEstimatorMock()
    : m_current_shooting_node()
    , m_step_duration(0.6)
    , m_shooting_nodes_per_step(125)
    , m_center_of_mass_height(0.5454)
    , m_current_stance_foot(-1)
    , m_counter(0)
    , m_previous_shooting_node(0)
{
}

void StateEstimatorMock::set_current_shooting_node(int current_shooting_node)
{
    m_current_shooting_node = current_shooting_node;
}

int StateEstimatorMock::get_current_shooting_node()
{
    return m_current_shooting_node;
}

double StateEstimatorMock::gaussian_dist(int currentStep, double targetValue, double sigma)
{
    double mean = 25.0; // in the middle of the weight shift, the highest speed has to be reached
    double amplitude = targetValue;
    double weightingFactor = amplitude * exp(-pow(currentStep - mean, 2) / (2 * pow(sigma, 2)));
    return weightingFactor;
}

march_shared_msgs::msg::CenterOfMass StateEstimatorMock::get_current_com()
{
    double max_y_com = 0.215; // How much can the CoM move
    double min_y_com = 0.115;
    double max_x_com = 0.1;
    double min_x_com = 0.0;
    double max_x_velocity = 0.25;
    double max_y_velocity = 0.25;

    int swing_duration = m_step_duration * m_shooting_nodes_per_step;
    int weight_shift_duration = m_shooting_nodes_per_step - swing_duration;
    march_shared_msgs::msg::CenterOfMass center_of_mass;

    // during swing phase
    if (m_current_stance_foot == -1 && m_current_shooting_node <= swing_duration) {
        center_of_mass.position.x = min_x_com;
        center_of_mass.position.y = max_y_com;
        center_of_mass.position.z = m_center_of_mass_height;
        // RCLCPP_INFO(rclcpp::get_logger(""), "pannenkoeks %f\n", center_of_mass.position.x);

        center_of_mass.velocity.x = 0.0;
        center_of_mass.velocity.y = 0.0;
        center_of_mass.velocity.z = 0.0;

    } else if (m_current_stance_foot == -1 && (m_current_shooting_node > swing_duration)
        && (m_current_shooting_node <= m_shooting_nodes_per_step)) {
        double shift_progress = static_cast<double>(m_current_shooting_node - swing_duration) / weight_shift_duration;

        // RCLCPP_INFO(rclcpp::get_logger(""), "shift progress is %f\n", shift_progress);
        if (m_counter == 0) {
            center_of_mass.position.x = min_x_com;
            center_of_mass.velocity.x = gaussian_dist((m_current_shooting_node - swing_duration), 0, 10.0);
            center_of_mass.velocity.y
                = gaussian_dist((m_current_shooting_node - swing_duration), -max_y_velocity / 2, 10.0);

        } else {
            center_of_mass.position.x = (1.0 - shift_progress) * min_x_com + shift_progress * max_x_com;
            center_of_mass.velocity.x = gaussian_dist((m_current_shooting_node - swing_duration), max_x_velocity, 10.0);
            center_of_mass.velocity.y
                = gaussian_dist((m_current_shooting_node - swing_duration), -max_y_velocity, 10.0);
        }
        center_of_mass.position.y = (1.0 - shift_progress) * max_y_com + shift_progress * min_y_com;
        center_of_mass.position.z = m_center_of_mass_height;

        center_of_mass.velocity.z = 0.0;

    } else if (m_current_stance_foot == 1 && m_current_shooting_node <= swing_duration) {
        center_of_mass.position.x = min_x_com;
        center_of_mass.position.y = min_y_com;
        center_of_mass.position.z = m_center_of_mass_height;
        // RCLCPP_INFO(rclcpp::get_logger(""), "pannenkoeks %f\n", center_of_mass.position.x);

        center_of_mass.velocity.x = 0.0;
        center_of_mass.velocity.y = 0.0;
        center_of_mass.velocity.z = 0.0;

    } else if (m_current_stance_foot == 1 && (m_current_shooting_node > swing_duration)
        && (m_current_shooting_node <= m_shooting_nodes_per_step)) {
        double shift_progress = static_cast<double>(m_current_shooting_node - swing_duration) / weight_shift_duration;
        if (m_counter == 0) {
            center_of_mass.position.x = min_x_com;
            center_of_mass.velocity.x = gaussian_dist((m_current_shooting_node - swing_duration), 0, 10.0);
            center_of_mass.velocity.y
                = gaussian_dist((m_current_shooting_node - swing_duration), max_y_velocity / 2, 10.0);

        } else {
            center_of_mass.position.x = (1.0 - shift_progress) * min_x_com + shift_progress * max_x_com;
            center_of_mass.velocity.x = gaussian_dist((m_current_shooting_node - swing_duration), max_x_velocity, 10.0);
            center_of_mass.velocity.y = gaussian_dist((m_current_shooting_node - swing_duration), max_y_velocity, 10.0);
        }
        // RCLCPP_INFO(rclcpp::get_logger(""), "shift progress is %f\n", shift_progress);

        center_of_mass.position.y = shift_progress * max_y_com + (1 - shift_progress) * min_y_com;
        center_of_mass.position.z = m_center_of_mass_height;

        center_of_mass.velocity.z = 0.0;
    }

    return center_of_mass;
}

geometry_msgs::msg::PointStamped StateEstimatorMock::get_current_zmp()
{
    double max_y_zmp = 0.33;
    double min_y_zmp = 0.0;
    double max_x_zmp = 0.1;
    double min_x_zmp = 0.0;

    int swing_duration = m_step_duration * m_shooting_nodes_per_step;
    int weight_shift_duration = m_shooting_nodes_per_step - swing_duration;
    geometry_msgs::msg::PointStamped zero_moment_point;
    zero_moment_point.header.frame_id = "map";

    // during swing phase
    if (m_current_stance_foot == -1 && m_current_shooting_node <= swing_duration) {
        zero_moment_point.point.x = min_x_zmp;
        zero_moment_point.point.y = max_y_zmp;
        zero_moment_point.point.z = 0.0;

    } else if (m_current_stance_foot == -1 && (m_current_shooting_node > swing_duration)
        && (m_current_shooting_node <= m_shooting_nodes_per_step)) {
        double shift_progress = static_cast<double>(m_current_shooting_node - swing_duration) / weight_shift_duration;
        if (m_counter == 0) {
            zero_moment_point.point.x = min_x_zmp;
        } else {
            zero_moment_point.point.x = (1.0 - shift_progress) * min_x_zmp + shift_progress * max_x_zmp;
        }
        zero_moment_point.point.y = (1.0 - shift_progress) * max_y_zmp;
        zero_moment_point.point.z = 0.0;

    } else if (m_current_stance_foot == 1 && m_current_shooting_node <= swing_duration) {
        zero_moment_point.point.x = min_x_zmp;
        zero_moment_point.point.y = min_y_zmp;
        zero_moment_point.point.z = 0.0;

    } else if (m_current_stance_foot == 1 && (m_current_shooting_node > swing_duration)
        && (m_current_shooting_node <= m_shooting_nodes_per_step)) {
        double shift_progress = static_cast<double>(m_current_shooting_node - swing_duration) / weight_shift_duration;
        if (m_counter == 0) {
            zero_moment_point.point.x = min_x_zmp;
        } else {
            zero_moment_point.point.x = (1.0 - shift_progress) * min_x_zmp + shift_progress * max_x_zmp;
        }
        zero_moment_point.point.y = shift_progress * max_y_zmp;
        zero_moment_point.point.z = 0.0;
    }
    return zero_moment_point;
}

geometry_msgs::msg::PoseArray StateEstimatorMock::get_previous_foot()
{
    geometry_msgs::msg::PoseArray foot_positions;
    foot_positions.header.frame_id = "map";
    geometry_msgs::msg::Pose right_foot;
    geometry_msgs::msg::Pose left_foot;

    if (m_current_stance_foot == -1) {
        right_foot.position.x = 0.0;
        right_foot.position.y = 0.0;
        right_foot.position.z = 0.0;
        foot_positions.poses.push_back(right_foot);

        left_foot.position.x = 0.0;
        left_foot.position.y = 0.33;
        left_foot.position.z = 0.0;
        foot_positions.poses.push_back(left_foot);

    } else if (m_current_stance_foot == 1) {
        right_foot.position.x = 0.0;
        right_foot.position.y = 0.0;
        right_foot.position.z = 0.0;
        foot_positions.poses.push_back(right_foot);

        left_foot.position.x = 0.0;
        left_foot.position.y = 0.33;
        left_foot.position.z = 0.0;
        foot_positions.poses.push_back(left_foot);
    }
    return foot_positions;
}

std_msgs::msg::Int32 StateEstimatorMock::get_current_stance_foot()
{
    std_msgs::msg::Int32 msg_stance_foot;
    if (m_current_shooting_node == m_shooting_nodes_per_step - 1
        && m_previous_shooting_node != m_current_shooting_node) {
        m_counter++;
        m_current_stance_foot = -m_current_stance_foot;
        // RCLCPP_INFO(rclcpp::get_logger(""), "stance foot %i\n", m_current_stance_foot);
    }
    m_previous_shooting_node = m_current_shooting_node;

    msg_stance_foot.data = m_current_stance_foot;
    return msg_stance_foot;
}

std_msgs::msg::Bool StateEstimatorMock::get_right_foot_ground()
{
    std_msgs::msg::Bool right_foot_on_ground;
    right_foot_on_ground.data = true;
    return right_foot_on_ground;
}

std_msgs::msg::Bool StateEstimatorMock::get_left_foot_ground()
{
    std_msgs::msg::Bool left_foot_on_ground;
    left_foot_on_ground.data = true;
    return left_foot_on_ground;
}
