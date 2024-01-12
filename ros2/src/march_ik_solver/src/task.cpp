#include "march_ik_solver/task.hpp"
#include "math.h"

#include "geometry_msgs/msg/point.hpp"
#include "march_shared_msgs/msg/node_jacobian.hpp"

Task::Task(uint8_t task_id, std::string task_name, uint8_t task_m, uint8_t task_n, std::vector<std::string> node_names)
{
    m_task_id = task_id;
    m_task_name = task_name;
    m_task_m = task_m;
    m_task_n = task_n;
    m_node_names = node_names;

    m_node = std::make_shared<rclcpp::Node>("task_" + m_task_name + "_client");
    m_client_node_position = m_node->create_client<march_shared_msgs::srv::GetNodePosition>("state_estimator/get_node_position");
    m_client_node_jacobian = m_node->create_client<march_shared_msgs::srv::GetNodeJacobian>("state_estimator/get_node_jacobian");
}

Eigen::VectorXd Task::solve()
{
    sendRequestNodePosition();
    sendRequestNodeJacobian();
    calculateJacobianInverse();

    Eigen::VectorXd error = calculateError();
    Eigen::VectorXd joint_velocities = Eigen::VectorXd::Zero(m_task_n);
    for (int i = 0; i < m_task_n; i++)
    {
        for (int j = 0; j < m_task_m; j++)
        {
            joint_velocities(i) += m_jacobian_inverse(i,j) * error(j);
        }
    }

    return joint_velocities;
}

Eigen::VectorXd Task::calculateError()
{
    Eigen::VectorXd pose_error =  (*m_desired_poses_ptr)[m_task_id] - m_current_pose;

    Eigen::VectorXd error = m_gain_p * pose_error; // + calculateIntegralError(pose_error) + calculateDerivativeError(pose_error);
    m_error_norm = error.norm();

    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Proportional error: %f, %f, %f, %f, %f, %f", error(0), error(1), error(2), error(3), error(4), error(5));

    return error;
}

Eigen::VectorXd Task::calculateIntegralError(const Eigen::VectorXd & error)
{
    Eigen::VectorXd integral_error = m_integral_error + error * m_dt;
    m_integral_error = integral_error;

    return m_gain_i * integral_error;
}

Eigen::VectorXd Task::calculateDerivativeError(const Eigen::VectorXd & error)
{
    Eigen::VectorXd derivative_error = (error - m_previous_error) / m_dt;
    m_previous_error = error;

    return m_gain_d * derivative_error;
}

std::string Task::getTaskName() const
{
    return m_task_name;
}

unsigned int Task::getTaskID() const
{
    return m_task_id;
}

int Task::getTaskM() const
{
    return m_task_m;
}

int Task::getTaskN() const
{
    return m_task_n;
}

double Task::getErrorNorm() const
{
    return m_error_norm;
}

void Task::setCurrentJointNamesPtr(std::vector<std::string> * current_joint_names_ptr)
{
    m_current_joint_names_ptr = current_joint_names_ptr;
}

void Task::setCurrentJointPositionsPtr(Eigen::VectorXd * current_joint_positions_ptr)
{
    m_current_joint_positions_ptr = current_joint_positions_ptr;
}

void Task::setDesiredPosesPtr(std::vector<Eigen::VectorXd> * desired_poses_ptr)
{
    m_desired_poses_ptr = desired_poses_ptr;
}

void Task::setCurrentPoses(const Eigen::VectorXd & current_pose)
{
    m_current_pose = current_pose;
}

void Task::setGainP(const float & gain_p)
{
    // Set the proportional gain
    m_gain_p = gain_p;
}

void Task::setGainI(const float & gain_i)
{
    // Set the integral gain
    m_gain_i = gain_i;
}

void Task::setGainD(const float & gain_d)
{
    // Set the derivative gain
    m_gain_d = gain_d;
}

void Task::setDt(const float & dt)
{
    // Set the time step
    m_dt = dt;
}

void Task::setDampingCoefficient(const float & damping_coefficient)
{
    // Set the damping coefficient
    m_damping_coefficient = damping_coefficient;
}

const Eigen::MatrixXd * Task::getJacobianPtr()
{
    return &m_jacobian;
}

const Eigen::MatrixXd * Task::getJacobianInversePtr()
{
    return &m_jacobian_inverse;
}

void Task::calculateJacobianInverse()
{
    Eigen::MatrixXd jacobian_transpose = m_jacobian.transpose();

    // If the system is underdetermined.
    if (m_task_m < m_task_n) {
        Eigen::MatrixXd damping_matrix = m_damping_coefficient * Eigen::MatrixXd::Identity(m_task_m, m_task_m);
        m_jacobian_inverse = jacobian_transpose * (m_jacobian * jacobian_transpose + damping_matrix).inverse();
    } else // if the system is overdetermined.
    {
        Eigen::MatrixXd damping_matrix = m_damping_coefficient * Eigen::MatrixXd::Identity(m_task_n, m_task_n);
        m_jacobian_inverse = (jacobian_transpose * m_jacobian + damping_matrix).inverse() * jacobian_transpose;
    }

    // m_jacobian_inverse = (jacobian_transpose * (m_jacobian * jacobian_transpose).inverse()) * (m_task_m < m_task_n) +
    //     ((jacobian_transpose * m_jacobian).inverse() * jacobian_transpose) * (m_task_m >= m_task_n);

    // Round the Jacobian inverse to zero if it is very small.
    for (int i = 0; i < m_task_n; i++)
    {
        for (int j = 0; j < m_task_m; j++)
        {
            if (abs(m_jacobian_inverse(i,j)) < 1e-6)
            {
                m_jacobian_inverse(i,j) = 0.0;
            }
        }
    }

    // Print the Jacobian inverse.
    // for (int i = 0; i < m_task_n; i++)
    // {
    //     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Jacobian inverse row %d: %f, %f, %f, %f, %f, %f", i, m_jacobian_inverse(i,0), m_jacobian_inverse(i,1), m_jacobian_inverse(i,2), m_jacobian_inverse(i,3), m_jacobian_inverse(i,4), m_jacobian_inverse(i,5));
    // }
}

void Task::sendRequestNodePosition()
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending request to get node position...");
    auto request = std::make_shared<march_shared_msgs::srv::GetNodePosition::Request>();
    request->node_names = m_node_names;
    
    // request->m_joint_names = *m_current_joint_names_ptr;
    request->joint_names = {"left_hip_aa", "left_hip_fe", "left_knee", "left_ankle", "right_hip_aa", "right_hip_fe", "right_knee", "right_ankle"};
    request->joint_positions = std::vector<double>(m_current_joint_positions_ptr->data(), m_current_joint_positions_ptr->data() + m_current_joint_positions_ptr->size());

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for service get_node_position...");
    while (!m_client_node_position->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending request to get node position...");
    auto result = m_client_node_position->async_send_request(request);

    if (rclcpp::spin_until_future_complete(m_node->get_node_base_interface(), result)
        == rclcpp::FutureReturnCode::SUCCESS) {

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received node position...");
        std::vector<double> current_pose_vector;

        for (auto node_position : result.get()->node_positions)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Node position: %f, %f, %f", node_position.x, node_position.y, node_position.z);
            current_pose_vector.push_back(node_position.x);
            current_pose_vector.push_back(node_position.y);
            current_pose_vector.push_back(node_position.z);
        }

        m_current_pose = Eigen::Map<Eigen::VectorXd>(current_pose_vector.data(), m_task_m);
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Current pose: %f, %f, %f, %f, %f, %f", m_current_pose(0), m_current_pose(1), m_current_pose(2), m_current_pose(3), m_current_pose(4), m_current_pose(5));
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service get_node_position");
    }
}

void Task::sendRequestNodeJacobian()
{
    auto request = std::make_shared<march_shared_msgs::srv::GetNodeJacobian::Request>();
    request->node_names = m_node_names;
    // TODO
    // request->m_joint_names = *m_current_joint_names_ptr;
    request->joint_names = {"left_hip_aa", "left_hip_fe", "left_knee", "left_ankle", "right_hip_aa", "right_hip_fe", "right_knee", "right_ankle"};
    request->joint_positions = std::vector<double>(m_current_joint_positions_ptr->data(), m_current_joint_positions_ptr->data() + m_current_joint_positions_ptr->size());

    while (!m_client_node_jacobian->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto result = m_client_node_jacobian->async_send_request(request);
    // std::vector<std::string> joint_names = {"left_hip_aa", "left_hip_fe", "left_knee", "left_ankle", "right_hip_aa", "right_hip_fe", "right_knee", "right_ankle"};

    if (rclcpp::spin_until_future_complete(m_node->get_node_base_interface(), result)
        == rclcpp::FutureReturnCode::SUCCESS) {

        Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(m_task_m, m_task_n);
        int total_rows = 0;

        for (auto & node_jacobian : result.get()->node_jacobians)
        {
            Eigen::MatrixXd jacobian_temp = Eigen::Map<Eigen::MatrixXd>(node_jacobian.jacobian.data(), node_jacobian.rows, node_jacobian.cols);
            for (int i = 0; i < node_jacobian.rows; i++)
            {
                long unsigned int counter = 0;
                for (long unsigned int j = 0; j < request->joint_names.size(); j++)
                {
                    if (node_jacobian.joint_names[counter] == request->joint_names[j])
                    {
                        jacobian(i + total_rows, j) = jacobian_temp(i, counter);
                        counter++;
                    }
                    else
                    {
                        jacobian(i + total_rows, j) = 0.0;
                    }
                }
            }
            total_rows += node_jacobian.rows;
        }

        m_jacobian = jacobian;
        // for (int i = 0; i < m_task_m; i++)
        // {
        //     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Jacobian row %d: %f, %f, %f, %f, %f, %f, %f, %f", i, m_jacobian(i,0), m_jacobian(i,1), m_jacobian(i,2), m_jacobian(i,3), m_jacobian(i,4), m_jacobian(i,5), m_jacobian(i,6), m_jacobian(i,7));
        // }
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service get_node_jacobian");
    }
}