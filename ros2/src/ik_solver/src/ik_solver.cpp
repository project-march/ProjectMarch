//
// Created by Sahand March 6th 2023
//
#include "ik_solver/ik_solver.hpp"
#include "rclcpp/rclcpp.hpp"

IkSolver::IkSolver()
{
}

/**
 * load and initialize the model which the ik uses to base the correct positions on.
 * @param urdf_filename The URDF of the exo with correct properties.
 */
void IkSolver::load_urdf_model(std::string urdf_filename)
{
    pinocchio::urdf::buildModel(urdf_filename, pinocchio::JointModelFreeFlyer(), m_model);
    m_model_data = pinocchio::Data(m_model);
    J_left_foot.resize(6, m_model.nv);
    J_left_foot.setZero();

    J_right_foot.resize(6, m_model.nv);
    J_right_foot.setZero();

    J_com.resize(6, m_model.nv);
    J_com.setZero();

    // Initialize the model
    m_joint_pos = pinocchio::neutral(m_model);

    m_joint_vel.resize(m_model.nv);

    for (int i = 1; i < m_model.names.size(); i++) {
        pinocchio::JointIndex index = m_model.getJointId(m_model.names[i]);
        if ((m_model.names[i].compare("right_knee") == 0) or (m_model.names[i].compare("left_knee") == 0)) {
            m_joint_pos(m_model.joints[index].idx_q()) = 0.1745;
        }
        if ((m_model.names[i].compare("right_ankle") == 0) or (m_model.names[i].compare("left_ankle") == 0)) {
            m_joint_pos(m_model.joints[index].idx_q()) = 0.1219;
        }
        if ((m_model.names[i].compare("right_hip_fe") == 0) or (m_model.names[i].compare("left_hip_fe") == 0)) {
            m_joint_pos(m_model.joints[index].idx_q()) = 0.12215;
        }
        if ((m_model.names[i].compare("right_hip_aa") == 0)) {
            m_joint_pos(m_model.joints[index].idx_q()) = -0.05;
        }
        if (m_model.names[i].compare("left_hip_aa") == 0) {
            m_joint_pos(m_model.joints[index].idx_q()) = -0.05;
        }
    }

    pinocchio::forwardKinematics(m_model, m_model_data, m_joint_pos, m_joint_vel);
    pinocchio::computeJointJacobians(m_model, m_model_data, m_joint_pos);
    pinocchio::updateFramePlacements(m_model, m_model_data);

    m_joint_lim_min = m_model.lowerPositionLimit;
    m_joint_lim_max = m_model.upperPositionLimit;

    m_body_com = pinocchio::centerOfMass(m_model, m_model_data, m_joint_pos);
}

/**
 * Set the configurations of the joints.
 * @param msg
 */
void IkSolver::set_joint_configuration(sensor_msgs::msg::JointState::SharedPtr msg)
{
    // We also update our map here
    for (long unsigned int i = 0; i < msg->name.size(); i++) {
        pinocchio::JointIndex index = m_model.getJointId(msg->name[i]);
        m_joint_pos[m_model.joints[index].idx_q()] = msg->position[i];
        m_pinocchio_to_march_joint_map.insert(std::pair<std::string, int>(msg->name[i], index));
    }
}

/**
 * Set the jacobian for the joint and com positions.
 * @return
 */
int IkSolver::set_jacobian()
{
    try {
        m_body_com = pinocchio::centerOfMass(m_model, m_model_data, m_joint_pos);

        pinocchio::JointIndex left_foot_index = m_model.getFrameId("L_toe");
        pinocchio::FrameIndex right_foot_index = m_model.getFrameId("R_toe");
        pinocchio::getJointJacobian(m_model, m_model_data, m_model.frames[left_foot_index].parent,
            pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_left_foot);
        pinocchio::getJointJacobian(m_model, m_model_data, m_model.frames[right_foot_index].parent,
            pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_right_foot);

        pinocchio::Data::Matrix3x angular_com_matrix(3, m_model.nv);
        angular_com_matrix.setZero();
        J_com << pinocchio::jacobianCenterOfMass(m_model, m_model_data, m_joint_pos), angular_com_matrix;
        return 0;
    } catch (...) {
        // Return Error CODE 1::JOINTS NOT INITIALIZED
        return 1;
    }
}

/**
 * Get the joints from the model.
 * @return
 */
int IkSolver::get_model_joints()
{
    try {
        return m_model.nv;
    } catch (...) {
        return -1;
    }
}

/**
 * Get the jacobian of teh model com.
 * @return
 */
pinocchio::Data::Matrix6x IkSolver::get_model_jacobian()
{
    return J_com;
}

/**
 * Initialize the solver function.
 */
void IkSolver::initialize_solver()
{
    m_qp_solver_ptr = std::make_shared<labrob::qpsolvers::QPSolverEigenWrapper<double>>(
        std::make_shared<labrob::qpsolvers::HPIPMQPSolver>(get_model_joints(), 6, get_model_joints()));
}

/**
 * The step that solves for velocities.
 * These velocities will alter be translated to positions.
 * @param state_current The current state of the exo.
 * @param state_desired The state we want to go to.
 * @param dt The delta time for this transformation
 * @param stance_foot the foot on which the exo is currently standing.
 * @return a trajectory of velocities.
 */
Eigen::VectorXd IkSolver::solve_for_velocity(state state_current, state state_desired, const double dt, int stance_foot)
{
    // WEIGHTS
    double left_weight = 0.1;
    double right_weight = 0.1;
    double CoM_weight = 1.0;
    double qdot_weight = 1e-6;
    // double base_weight = 1;

    double CoM_gains = 1.0;
    double left_gains = 1.0;
    double right_gains = 1.0;

    // Get the error vectors
    // Here, we assume the jacobians have already been set
    // The error is equal to the desired state, as we express them in local coordinates
    Eigen::VectorXd left_foot_error = state_desired.left_foot_pose;
    left_foot_error.segment(3, 3)
        = angleSignedDistance(state_desired.left_foot_pose.segment(3, 3), state_current.left_foot_pose.segment(3, 3));
    Eigen::VectorXd right_foot_error = state_desired.right_foot_pose;
    right_foot_error.segment(3, 3)
        = angleSignedDistance(state_desired.right_foot_pose.segment(3, 3), state_current.right_foot_pose.segment(3, 3));

    Eigen::VectorXd com_pos_error = state_desired.com_pos;
    com_pos_error.segment(0, 3) = state_desired.com_pos.segment(0, 3); // - m_body_com;

    // Set up the cost function
    // H: The quadratic term of the cost function
    // F: The linear term of the cost function
    // We should see this as a^2 + 2ab + b^2
    // where:
    // a^2 + b^2 is the quadratic term
    // 2ab is the linear term
    // We ignore b^2 as it is a constant value. Thus, it has no impact on an optimization problem.
    Eigen::MatrixXd cost_H = qdot_weight * Eigen::MatrixXd::Identity(m_model.nv, m_model.nv);
    Eigen::VectorXd cost_F = Eigen::VectorXd::Zero(m_model.nv);

    // NOTE::: CONSIDER ADDING THE SINGULARITY ROBUST REGULARIZATION TERM
    // [NOTE]: ADD THE ILNEAR TERM USING COM VELOCITIES
    cost_H += left_weight
        * (J_left_foot.transpose() * J_left_foot + Eigen::MatrixXd::Identity(m_model.nv, m_model.nv) * 0.001);
    cost_F += -left_weight * J_left_foot.transpose() * (state_desired.left_foot_vel + left_gains * left_foot_error);

    cost_H += right_weight
        * (J_right_foot.transpose() * J_right_foot + Eigen::MatrixXd::Identity(m_model.nv, m_model.nv) * 0.001);
    cost_F
        += -right_weight * J_right_foot.transpose() * (state_desired.right_foot_vel + right_gains * right_foot_error);

    cost_H += CoM_weight * (J_com.transpose() * J_com + Eigen::MatrixXd::Identity(m_model.nv, m_model.nv) * 0.0001);
    cost_F += -CoM_weight * J_com.transpose() * (state_desired.com_vel + CoM_gains * com_pos_error);

    // Add the constraints
    Eigen::MatrixXd constrained_joints = Eigen::MatrixXd::Identity(m_model.nv, m_model.nv);
    int free_joint_start = 0;

    Eigen::VectorXd joint_upper_lim = (m_joint_lim_max.tail(m_model.nv) - m_joint_pos.tail(m_model.nv)) / dt;
    Eigen::VectorXd joint_lower_lim = (m_joint_lim_min.tail(m_model.nv) - m_joint_pos.tail(m_model.nv)) / dt;
    joint_upper_lim.segment(0, 3) << 1e-4 * Eigen::VectorXd::Ones(3);
    joint_lower_lim.segment(0, 3) << -1e-4 * Eigen::VectorXd::Ones(3);

    joint_upper_lim.segment(0, 1) << 1;
    joint_lower_lim.segment(0, 1) << -1;

    joint_upper_lim.segment(3, 4) << 1e-4 * Eigen::VectorXd::Ones(4);
    joint_lower_lim.segment(3, 4) << -1e-4 * Eigen::VectorXd::Ones(4);
    // dummy equality constraint
    // This is needed to set one of the feet to a fixed position
    // Equal to structure A*q_dot = b
    // We set these to zero so we know we cannot influence this q_dot
    Eigen::MatrixXd A_dummy = Eigen::MatrixXd::Zero(6, m_model.nv);
    Eigen::VectorXd b_dummy = Eigen::VectorXd::Zero(6);

    // We now swap the dummy foot based on the fixed Jacobian.
    if (stance_foot == -1) {
        A_dummy = J_left_foot;
    } else {
        A_dummy = J_right_foot;
    }

    m_qp_solver_ptr->solve(cost_H, cost_F, A_dummy, b_dummy, constrained_joints, joint_lower_lim, joint_upper_lim);
    Eigen::VectorXd velocity_trajectory = m_qp_solver_ptr->get_solution();

    return velocity_trajectory; // CHANGE 50 to [DOF-6], so [14-6]
}

/**
 * This functions translates the velocity trajectory to a position based trajectory.
 * @param velocity velocity output of the solving step.
 * @param dt delta t of the trajectory.
 * @return
 */
Eigen::VectorXd IkSolver::velocity_to_pos(Eigen::VectorXd& velocity, const double dt)
{
    Eigen::VectorXd new_position = Eigen::VectorXd::Zero(velocity.size() + 1);
    std::map<std::string, int>::iterator it;

    pinocchio::JointIndex index = 0;
    for (int i = 1; i < m_model.names.size(); i++) {
        index = m_model.getJointId(m_model.names[i]);
        if (index < m_model.nv) {
            new_position[m_model.joints[index].idx_q()]
                = m_joint_pos[m_model.joints[index].idx_q()] + dt * velocity[m_model.joints[index].idx_v()];
        }
    }
    return new_position;
}

/**
 * Get the model the ik solver uses.
 * @return
 */
const pinocchio::Model IkSolver::get_model()
{
    return m_model;
}

/**
 * Get the data from the model.
 * @return
 */
const pinocchio::Data IkSolver::get_data()
{
    return m_model_data;
}

/**
 * Get the current joint positions.
 * @return
 */
std::vector<double> IkSolver::get_joint_pos()
{
    std::vector<double> to_return;
    for (int i = 0; i < m_joint_pos.size(); i++) {
        to_return.push_back(m_joint_pos(i));
    }
    return to_return;
}

/**
 * Get the current joint velocities.
 * @return
 */
std::vector<double> IkSolver::get_joint_vel()
{
    return std::vector<double>(m_joint_vel.data(), m_joint_vel.data() + m_joint_vel.size());
}

/**
 * Set the current state of the model.
 */
void IkSolver::set_current_state()
{
    pinocchio::forwardKinematics(m_model, m_model_data, m_joint_pos, m_joint_vel);
    pinocchio::computeJointJacobians(m_model, m_model_data, m_joint_pos);
    pinocchio::updateFramePlacements(m_model, m_model_data);
    pinocchio::FrameIndex left_foot_index = m_model.getFrameId("L_toe");
    pinocchio::FrameIndex right_foot_index = m_model.getFrameId("R_toe");

    m_current_state.left_foot_pose << m_model_data.oMf[left_foot_index].translation(),
        pinocchio::rpy::matrixToRpy(m_model_data.oMf[left_foot_index].rotation());
    m_current_state.right_foot_pose << m_model_data.oMf[right_foot_index].translation(),
        pinocchio::rpy::matrixToRpy(m_model_data.oMf[right_foot_index].rotation());

    m_current_state.com_pos << pinocchio::centerOfMass(m_model, m_model_data, m_joint_pos), 0.0, 0.0, 0.0;
}

/**
 * Get the current state of the model.
 * @return
 */
const state IkSolver::get_state()
{
    return m_current_state;
}
