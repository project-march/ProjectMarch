#include "march_ik_solver/task.hpp"
#include "math.h"

#include "geometry_msgs/msg/point.hpp"
#include "march_shared_msgs/msg/node_jacobian.hpp"

Task::Task(const std::string& task_name, const std::string& reference_frame, const unsigned int& workspace_dim,
    const unsigned int& configuration_dim, const float& dt)
{
    m_task_name = task_name;
    m_reference_frame = reference_frame;
    setTaskM(workspace_dim);
    setTaskN(configuration_dim);
    m_dt = dt;
    m_previous_error = Eigen::VectorXd::Zero(workspace_dim);
    m_integral_error = Eigen::VectorXd::Zero(workspace_dim);

    // Initialize ROS node and service clients.
    m_node = std::make_shared<rclcpp::Node>("task_" + m_task_name + "_client");
    m_client_node_position
        = m_node->create_client<march_shared_msgs::srv::GetNodePosition>("state_estimation/get_node_position");
    m_client_node_jacobian
        = m_node->create_client<march_shared_msgs::srv::GetNodeJacobian>("state_estimation/get_node_jacobian");
}

void Task::setTaskName(const std::string& task_name)
{
    m_task_name = task_name;
}

void Task::setNodeNames(const std::vector<std::string>& node_names)
{
    m_node_names = node_names;
}

void Task::setTaskM(const unsigned int& task_m)
{
    m_task_m = task_m;
    
    // Initialize gains as diagonal matrices
    m_gain_p = Eigen::MatrixXd::Zero(task_m, task_m);
    m_gain_d = Eigen::MatrixXd::Zero(task_m, task_m);
    m_gain_i = Eigen::MatrixXd::Zero(task_m, task_m);
    for (unsigned int i = 0; i < task_m; i++) {
        m_gain_p.diagonal()[i] = 1.0;
        m_gain_d.diagonal()[i] = 0.0;
        m_gain_i.diagonal()[i] = 0.0;
    }
}

void Task::setTaskN(const unsigned int& task_n)
{
    m_task_n = task_n;

    // Initialize damping identity matrix for singularity-robustness
    m_damping_identity = Eigen::MatrixXd::Identity(m_task_m, task_n);
}

void Task::setDt(const float& dt)
{
    m_dt = dt;
}

void Task::setGainP(const std::vector<double>& gain_p)
{
    m_nonzero_gain_p_indices.clear();
    for (unsigned long int i = 0; i < gain_p.size(); i++) {
        m_gain_p.diagonal()[i] = gain_p[i];
        if (gain_p[i] > 0.0) {
            m_nonzero_gain_p_indices.push_back(i);
        }
    }
}

void Task::setGainD(const std::vector<double>& gain_d)
{
    for (unsigned long int i = 0; i < gain_d.size(); i++) {
        m_gain_d.diagonal()[i] = gain_d[i];
    }
}

void Task::setGainI(const std::vector<double>& gain_i)
{
    for (unsigned long int i = 0; i < gain_i.size(); i++) {
        m_gain_i.diagonal()[i] = gain_i[i];
    }
}

void Task::setDampingCoefficient(const float& damping_coefficient)
{
    m_damping_coefficient = damping_coefficient;
    m_damping_identity.noalias() = damping_coefficient * m_damping_identity;
}

void Task::setDesiredTask(const Eigen::VectorXd& desired_task)
{
    m_desired_task = desired_task;
}

Eigen::VectorXd Task::solveTask()
{
    Eigen::VectorXd error = calculateError();
    Eigen::VectorXd joint_velocities = Eigen::VectorXd::Zero(m_task_n);
    joint_velocities.noalias() = m_jacobian_inverse * error;
    return joint_velocities;
}

Eigen::VectorXd Task::calculateError()
{
    Eigen::VectorXd error, pid_error;

    error.noalias() = m_desired_task - m_current_task;
    m_previous_error = error;
    
    m_error_norm = 0.0;
    for (const auto& nonzero_gain_p_idx : m_nonzero_gain_p_indices) {
        m_error_norm += pow(error(nonzero_gain_p_idx), 2);
    }
    m_error_norm = sqrt(m_error_norm);

    pid_error.noalias()
        = m_gain_p * error; // + calculateIntegralError(error) + calculateDerivativeError(error);
    return pid_error;
}

Eigen::VectorXd Task::calculateIntegralError(const Eigen::VectorXd& error)
{
    Eigen::VectorXd integral_error;
    m_integral_error.noalias() = m_integral_error + error * m_dt;
    integral_error.noalias() = m_gain_i * m_integral_error;
    return integral_error;
}

Eigen::VectorXd Task::calculateDerivativeError(const Eigen::VectorXd& error)
{
    Eigen::VectorXd derivative_error;
    derivative_error.noalias() = m_gain_d * ((error - m_previous_error) / m_dt);
    return derivative_error;
}

void Task::requestCurrentTask()
{
    sendRequestNodePosition();
    sendRequestNodeJacobian();
    calculateJacobianInverse();
}

void Task::calculateJacobianInverse()
{
    Eigen::MatrixXd damped_jacobian = Eigen::MatrixXd::Zero(m_task_m, m_task_n);
    damped_jacobian.noalias() = m_jacobian + m_damping_identity;
    m_jacobian_inverse.noalias() = damped_jacobian.completeOrthogonalDecomposition().pseudoInverse();
}

std::vector<std::string> Task::getNodeNames() const
{
    return m_node_names;
}

std::string Task::getTaskName() const
{
    return m_task_name;
}

unsigned int Task::getTaskM() const
{
    return m_task_m;
}

unsigned int Task::getTaskN() const
{
    return m_task_n;
}

double Task::getErrorNorm() const
{
    return m_error_norm;
}

Eigen::VectorXd Task::getDesiredTask() const
{
    return m_desired_task;
}

Eigen::MatrixXd Task::getNullspaceProjection() const
{
    Eigen::MatrixXd nullspace_projection;
    nullspace_projection.noalias() = Eigen::MatrixXd::Identity(m_task_n, m_task_n) - m_jacobian_inverse * m_jacobian;
    return nullspace_projection;
}

const std::vector<std::string>* Task::getJointNamesPtr() const
{
    return m_joint_names_ptr;
}

const Eigen::VectorXd* Task::getCurrentJointPositionsPtr() const
{
    return m_current_joint_positions_ptr;
}

void Task::setJointNamesPtr(std::vector<std::string>* joint_names_ptr)
{
    m_joint_names_ptr = joint_names_ptr;
}

void Task::setCurrentJointPositionsPtr(Eigen::VectorXd* current_joint_positions_ptr)
{
    m_current_joint_positions_ptr = current_joint_positions_ptr;
}

void Task::setCurrentTask(const Eigen::VectorXd& current_task)
{
    m_current_task = current_task;
}

void Task::setJacobian(const Eigen::MatrixXd& jacobian)
{
    m_jacobian = jacobian;
}

void Task::setUnitTest(const bool& unit_test)
{
    m_unit_test = unit_test;
}

Eigen::MatrixXd Task::getJacobian() const
{
    return m_jacobian;
}

Eigen::MatrixXd Task::getJacobianInverse() const
{
    return m_jacobian_inverse;
}

Eigen::Vector3d Task::calculateEulerAnglesFromQuaternion(const geometry_msgs::msg::Quaternion& quaternion)
{
    Eigen::Vector3d euler_angles = Eigen::Vector3d::Zero();
    double norm, norm_sine_2;

    // Avoid division by zero
    if (quaternion.w >= 1.0) {
        return euler_angles;
    }

    norm = 2 * acos(quaternion.w);
    norm_sine_2 = sin(0.5 * norm);
    euler_angles.x() = quaternion.x / norm_sine_2;
    euler_angles.y() = quaternion.y / norm_sine_2;
    euler_angles.z() = quaternion.z / norm_sine_2;

    return euler_angles;
}

void Task::sendRequestNodePosition()
{
    auto position_request = std::make_shared<march_shared_msgs::srv::GetNodePosition::Request>();
    position_request->reference_frame = m_reference_frame;
    position_request->node_names = m_node_names;
    position_request->joint_names = *m_joint_names_ptr;
    position_request->joint_positions = std::vector<double>(m_current_joint_positions_ptr->data(),
        m_current_joint_positions_ptr->data() + m_current_joint_positions_ptr->size());

    while (!m_client_node_position->wait_for_service(std::chrono::milliseconds(m_service_timeout))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("ik_solver_node"), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_DEBUG(rclcpp::get_logger("ik_solver_node"), "service not available, waiting again...");
    }

    auto result = m_client_node_position->async_send_request(position_request);
    if (rclcpp::spin_until_future_complete(m_node->get_node_base_interface(), result)
        == rclcpp::FutureReturnCode::SUCCESS) {
        std::vector<double> current_task_vector;
        for (auto node_pose : result.get()->node_poses) {
            // Push back position
            current_task_vector.push_back(node_pose.position.x);
            current_task_vector.push_back(node_pose.position.y);
            current_task_vector.push_back(node_pose.position.z);

            // Push back orientation
            Eigen::Vector3d euler_angles = calculateEulerAnglesFromQuaternion(node_pose.orientation);
            current_task_vector.push_back(euler_angles.x());
            current_task_vector.push_back(euler_angles.y());
            current_task_vector.push_back(euler_angles.z());
        }
        m_current_task = Eigen::Map<Eigen::VectorXd>(current_task_vector.data(), m_task_m);
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("ik_solver_node"), "Failed to call service get_node_position");
    }
}

void Task::sendRequestNodeJacobian()
{
    auto position_jacobian_request = std::make_shared<march_shared_msgs::srv::GetNodeJacobian::Request>();
    position_jacobian_request->reference_frame = m_reference_frame;
    position_jacobian_request->node_names = m_node_names;
    position_jacobian_request->joint_names = *m_joint_names_ptr;
    position_jacobian_request->joint_positions = std::vector<double>(m_current_joint_positions_ptr->data(),
        m_current_joint_positions_ptr->data() + m_current_joint_positions_ptr->size());

    while (!m_client_node_jacobian->wait_for_service(std::chrono::milliseconds(m_service_timeout))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("ik_solver_node"), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_DEBUG(rclcpp::get_logger("ik_solver_node"), "service not available, waiting again...");
    }

    auto result = m_client_node_jacobian->async_send_request(position_jacobian_request);
    if (rclcpp::spin_until_future_complete(m_node->get_node_base_interface(), result)
        == rclcpp::FutureReturnCode::SUCCESS) {

        m_jacobian.noalias() = Eigen::MatrixXd::Zero(m_task_m, m_task_n);
        const unsigned int JACOBIAN_SIZE = result.get()->node_jacobians.size();
        const unsigned int WORKSPACE_DIM = m_task_m / JACOBIAN_SIZE;
        const unsigned int CONFIGURATION_DIM = m_task_n / JACOBIAN_SIZE;

        for (long unsigned int i = 0; i < JACOBIAN_SIZE; i++) {
            const unsigned int jacobian_rows = result.get()->node_jacobians[i].rows;
            const unsigned int jacobian_cols = result.get()->node_jacobians[i].cols;
            m_jacobian.block(i * WORKSPACE_DIM, i * CONFIGURATION_DIM, jacobian_rows, jacobian_cols).noalias()
                = Eigen::Map<Eigen::MatrixXd>(
                    result.get()->node_jacobians[i].jacobian.data(), jacobian_rows, jacobian_cols);
        }
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("ik_solver_node"), "Failed to call service get_node_jacobian");
    }
}