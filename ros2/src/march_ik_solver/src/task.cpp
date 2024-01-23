#include "march_ik_solver/task.hpp"
#include "math.h"

#include "geometry_msgs/msg/point.hpp"
#include "march_shared_msgs/msg/node_jacobian.hpp"

Task::Task(unsigned int task_id, std::string task_name, unsigned int task_m, unsigned int task_n, std::vector<std::string> node_names)
{
    m_task_id = task_id;
    m_task_name = task_name;
    m_task_m = task_m;
    m_task_n = task_n;
    m_node_names = node_names;

    m_node = std::make_shared<rclcpp::Node>("task_" + m_task_name + "_client");
    m_client_node_position = m_node->create_client<march_shared_msgs::srv::GetNodePosition>("state_estimation/get_node_position");
    m_client_node_jacobian = m_node->create_client<march_shared_msgs::srv::GetNodeJacobian>("state_estimation/get_node_jacobian");
}

Eigen::VectorXd Task::solve()
{
    if (!m_unit_test)
    {
        sendRequestNodePosition();
        sendRequestNodeJacobian();
    }
    calculateJacobianInverse();

    Eigen::VectorXd error = calculateError();
    Eigen::VectorXd joint_velocities = Eigen::VectorXd::Zero(m_task_n);
    joint_velocities.noalias() = m_jacobian_inverse * error;
    return joint_velocities;
}

Eigen::VectorXd Task::calculateError()
{
    Eigen::VectorXd pose_error, error;
    pose_error.noalias() =  (*m_desired_poses_ptr)[m_task_id] - m_current_pose;
    // Eigen::VectorXd gain_p_matrix = Eigen::VectorXd::Zero(m_task_m);
    // gain_p_vector << m_gain_p, m_gain_p * 5.0, m_gain_p, m_gain_p, m_gain_p * 5.0, m_gain_p; // TODO
    Eigen::DiagonalMatrix<double, Eigen::Dynamic> gain_p_matrix(m_task_m);
    gain_p_matrix.diagonal() << m_gain_p, m_gain_p * 5.0, m_gain_p, m_gain_p, m_gain_p * 5.0, m_gain_p;
    error.noalias() = gain_p_matrix * pose_error; // + calculateIntegralError(pose_error) + calculateDerivativeError(pose_error);
    m_error_norm = error.norm();
    return error;
}

Eigen::VectorXd Task::calculateIntegralError(const Eigen::VectorXd & error)
{
    Eigen::VectorXd integral_error;
    integral_error.noalias() = m_integral_error + error * m_dt;
    m_integral_error = integral_error;

    return m_gain_i * integral_error;
}

Eigen::VectorXd Task::calculateDerivativeError(const Eigen::VectorXd & error)
{
    Eigen::VectorXd derivative_error = (error - m_previous_error) / m_dt;
    m_previous_error = error;

    return m_gain_d * derivative_error;
}

void Task::calculateJacobianInverse()
{
    Eigen::MatrixXd jacobian_transpose = m_jacobian.transpose();

    // If the system is overdetermined.
    if (m_task_m < m_task_n) {
        Eigen::MatrixXd damping_matrix = m_damping_coefficient * Eigen::MatrixXd::Identity(m_task_m, m_task_m);
        m_jacobian_inverse.noalias() = jacobian_transpose * (m_jacobian * jacobian_transpose + damping_matrix).inverse();
    } else if (m_task_m > m_task_n) // if the system is undetermined.
    {
        Eigen::MatrixXd damping_matrix = m_damping_coefficient * Eigen::MatrixXd::Identity(m_task_n, m_task_n);
        m_jacobian_inverse.noalias() = (jacobian_transpose * m_jacobian + damping_matrix).inverse() * jacobian_transpose;
    }
    else // if the system is determined.
    {
        Eigen::MatrixXd damping_matrix = m_damping_coefficient * Eigen::MatrixXd::Identity(m_task_m, m_task_n);
        m_jacobian_inverse.noalias() = (m_jacobian + damping_matrix).inverse();
    }

    // m_jacobian_inverse = (jacobian_transpose * (m_jacobian * jacobian_transpose).inverse()) * (m_task_m < m_task_n) +
    //     ((jacobian_transpose * m_jacobian).inverse() * jacobian_transpose) * (m_task_m >= m_task_n);

    // Round the Jacobian inverse to zero if it is very small.
    // for (unsigned int i = 0; i < m_task_n; i++)
    // {
    //     for (unsigned int j = 0; j < m_task_m; j++)
    //     {
    //         if (abs(m_jacobian_inverse(i,j)) < 1e-6)
    //         {
    //             m_jacobian_inverse(i,j) = 0.0;
    //         }
    //     }
    // }
}

std::string Task::getTaskName() const
{
    return m_task_name;
}

unsigned int Task::getTaskID() const
{
    return m_task_id;
}

unsigned int Task::getTaskM() const
{
    return m_task_m;
}

unsigned int Task::getTaskN() const
{
    return m_task_n;
}

void Task::setTaskName(const std::string & task_name)
{
    m_task_name = task_name;
}

void Task::setTaskID(const unsigned int & task_id)
{
    m_task_id = task_id;
}

void Task::setTaskM(const unsigned int & task_m)
{
    m_task_m = task_m;
}

void Task::setTaskN(const unsigned int & task_n)
{
    m_task_n = task_n;
}

void Task::setNodeNames(const std::vector<std::string> & node_names)
{
    m_node_names = node_names;
}

std::vector<std::string> Task::getNodeNames() const
{
    return m_node_names;
}

double Task::getErrorNorm() const
{
    return m_error_norm;
}

const Eigen::VectorXd * Task::getCurrentJointPositionsPtr() const
{
    return m_current_joint_positions_ptr;
}

const std::vector<std::string> * Task::getCurrentJointNamesPtr() const
{
    return m_current_joint_names_ptr;
}

const std::vector<Eigen::VectorXd> * Task::getDesiredPosesPtr() const
{
    return m_desired_poses_ptr;
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

void Task::setCurrentPose(const Eigen::VectorXd & current_pose)
{
    m_current_pose = current_pose;
}

void Task::setJacobian(const Eigen::MatrixXd & jacobian)
{
    m_jacobian = jacobian;
}

void Task::setGainP(const float & gain_p)
{
    m_gain_p = gain_p;
}

void Task::setGainI(const float & gain_i)
{
    m_gain_i = gain_i;
}

void Task::setGainD(const float & gain_d)
{
    m_gain_d = gain_d;
}

void Task::setDt(const float & dt)
{
    m_dt = dt;
}

void Task::setDampingCoefficient(const float & damping_coefficient)
{
    m_damping_coefficient = damping_coefficient;
}

void Task::setUnitTest(const bool & unit_test)
{
    m_unit_test = unit_test;
}

const Eigen::MatrixXd * Task::getJacobianPtr()
{
    return &m_jacobian;
}

const Eigen::MatrixXd * Task::getJacobianInversePtr()
{
    return &m_jacobian_inverse;
}

void Task::sendRequestNodePosition()
{
    RCLCPP_DEBUG(rclcpp::get_logger("ik_solver_node"), "Sending request to get node position...");
    auto request = std::make_shared<march_shared_msgs::srv::GetNodePosition::Request>();
    request->node_names = m_node_names;
    RCLCPP_DEBUG(rclcpp::get_logger("ik_solver_node"), "Node names: %s, %s", request->node_names[0].c_str(), request->node_names[1].c_str());
    
    // request->m_joint_names = *m_current_joint_names_ptr;
    request->joint_names = {"left_hip_aa", "left_hip_fe", "left_knee", "left_ankle", "right_hip_aa", "right_hip_fe", "right_knee", "right_ankle"};
    request->joint_positions = std::vector<double>(m_current_joint_positions_ptr->data(), m_current_joint_positions_ptr->data() + m_current_joint_positions_ptr->size());
    RCLCPP_DEBUG(rclcpp::get_logger("ik_solver_node"), "Joint names: %s, %s, %s, %s, %s, %s, %s, %s", 
        request->joint_names[0].c_str(), request->joint_names[1].c_str(), request->joint_names[2].c_str(), request->joint_names[3].c_str(),
        request->joint_names[4].c_str(), request->joint_names[5].c_str(), request->joint_names[6].c_str(), request->joint_names[7].c_str());
    RCLCPP_DEBUG(rclcpp::get_logger("ik_solver_node"), "Joint positions: %f, %f, %f, %f, %f, %f, %f, %f",
        request->joint_positions[0], request->joint_positions[1], request->joint_positions[2], request->joint_positions[3],
        request->joint_positions[4], request->joint_positions[5], request->joint_positions[6], request->joint_positions[7]);

    RCLCPP_DEBUG(rclcpp::get_logger("ik_solver_node"), "Waiting for service get_node_position...");
    while (!m_client_node_position->wait_for_service(std::chrono::milliseconds(10))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("ik_solver_node"), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_DEBUG(rclcpp::get_logger("ik_solver_node"), "service not available, waiting again...");
    }

    RCLCPP_DEBUG(rclcpp::get_logger("ik_solver_node"), "Sending request to get node position...");
    auto result = m_client_node_position->async_send_request(request);

    if (rclcpp::spin_until_future_complete(m_node->get_node_base_interface(), result)
        == rclcpp::FutureReturnCode::SUCCESS) {

        RCLCPP_DEBUG(rclcpp::get_logger("ik_solver_node"), "Received node position...");
        std::vector<double> current_pose_vector;

        for (auto node_position : result.get()->node_positions)
        {
            RCLCPP_INFO(rclcpp::get_logger("ik_solver_node"), "Node position: %f, %f, %f", node_position.x, node_position.y, node_position.z);
            current_pose_vector.push_back(node_position.x);
            current_pose_vector.push_back(node_position.y);
            current_pose_vector.push_back(node_position.z);
        }

        m_current_pose = Eigen::Map<Eigen::VectorXd>(current_pose_vector.data(), m_task_m);
        // RCLCPP_DEBUG(rclcpp::get_logger("ik_solver_node"), "Current pose: %f, %f, %f, %f, %f, %f", m_current_pose(0), m_current_pose(1), m_current_pose(2), m_current_pose(3), m_current_pose(4), m_current_pose(5));
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("ik_solver_node"), "Failed to call service get_node_position");
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
            RCLCPP_ERROR(rclcpp::get_logger("ik_solver_node"), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_DEBUG(rclcpp::get_logger("ik_solver_node"), "service not available, waiting again...");
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
                    // TODO: Use BottomRightCorner, TopRightCorner, BottomLeftCorner, TopLeftCorner
                }
            }
            total_rows += node_jacobian.rows;
        }

        m_jacobian = jacobian;
        // for (unsigned int i = 0; i < m_task_m; i++)
        // {
        //     RCLCPP_DEBUG(rclcpp::get_logger("ik_solver_node"), "Jacobian row %d: %f, %f, %f, %f, %f, %f, %f, %f", i, m_jacobian(i,0), m_jacobian(i,1), m_jacobian(i,2), m_jacobian(i,3), m_jacobian(i,4), m_jacobian(i,5), m_jacobian(i,6), m_jacobian(i,7));
        // }
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("ik_solver_node"), "Failed to call service get_node_jacobian");
    }
}