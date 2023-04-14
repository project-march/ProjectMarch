//
// Created by Sahand March 6th 2023
//
#include "ik_solver/ik_solver.hpp"

IkSolver::IkSolver()
{
}

void IkSolver::load_urdf_model(std::string urdf_filename)
{
    pinocchio::urdf::buildModel(urdf_filename, m_model);
    // pinocchio::buildModels::manipulator(m_model);
    m_model_data = pinocchio::Data(m_model);
    J_left_foot.resize(6, m_model.nv);
    J_left_foot.setZero();

    J_right_foot.resize(6, m_model.nv);
    J_right_foot.setZero();

    J_com.resize(6, m_model.nv);
    J_com.setZero();

    // Initialize the model
    m_joint_pos = pinocchio::neutral(m_model);
    pinocchio::forwardKinematics(m_model, m_model_data, m_joint_pos);
    pinocchio::updateFramePlacements(m_model, m_model_data);

}

void IkSolver::set_joint_configuration(sensor_msgs::msg::JointState::SharedPtr msg)
{
    for (int i = 0; i < msg->name.size(); i++) {
        pinocchio::FrameIndex index = m_model.getJointId(msg->name[i]);
        m_model.joints[index].setIndexes(index, msg->position[i], msg->velocity[i]);
    }
}

int IkSolver::set_jacobian()
{
    try {
        pinocchio::FrameIndex left_foot_index = m_model.getFrameId("foot_left");
        pinocchio::FrameIndex right_foot_index = m_model.getFrameId("foot_left");
        pinocchio::computeJointJacobian(
            m_model, m_model_data, m_joint_pos, m_model.frames[left_foot_index].parent, J_left_foot);
        pinocchio::computeJointJacobian(
            m_model, m_model_data, m_joint_pos, m_model.frames[right_foot_index].parent, J_right_foot);
        pinocchio::Data::Matrix3x angular_com_matrix(3, m_model.nv);
        angular_com_matrix.setZero();
        J_com << pinocchio::jacobianCenterOfMass(m_model, m_model_data, m_joint_pos), angular_com_matrix;
        return 0;
    } catch (...) {
        // Return Error CODE 1::JOINTS NOT INITIALIZED
        return 1;
    }
}

int IkSolver::get_model_joints()
{
    try {
        return m_model.nv;
    } catch (...) {
        return -1;
    }
}

pinocchio::Data::Matrix6x IkSolver::get_model_jacobian()
{
    return J_com;
}

void IkSolver::initialize_solver()
{
    m_qp_solver_ptr = std::make_shared<labrob::qpsolvers::QPSolverEigenWrapper<double>>(
        std::make_shared<labrob::qpsolvers::HPIPMQPSolver>(get_model_joints(), 6, get_model_joints()));
}

Eigen::VectorXd IkSolver::solve_for_velocity(state state_current, state state_desired, int stance_foot)
{
    // We put the weights here, but of course we can remove them later
    // WEIGHTS
    double left_weight = 0.1;
    double right_weight = 0.1;
    double CoM_weight = 1;
    double qdot_weight = 1e-6;
    double base_weight = 1;

    double CoM_gains = 0.5;
    double left_gains = 1;
    double right_gains = 1;

    Eigen::MatrixXd J_com_copy;
    J_com_copy.resize(6, m_model.nv);
    J_com_copy<< J_com, 0.0, 0.0, 0.0;

    // Get the error vectors
    // Here, we assume the jacobians have already been set
    Eigen::VectorXd left_foot_error = state_desired.left_foot_pose - state_current.left_foot_pose;
    left_foot_error.segment(0, 3)
        = angleSignedDistance(state_desired.left_foot_pose.segment(0, 3), state_current.left_foot_pose.segment(0, 3));
    Eigen::VectorXd right_foot_error = state_desired.right_foot_pose - state_current.right_foot_pose;
    right_foot_error.segment(0, 3)
        = angleSignedDistance(state_desired.left_foot_pose.segment(0, 3), state_current.left_foot_pose.segment(0, 3));
    Eigen::VectorXd com_pos_error = state_desired.com_pos - state_current.com_pos;

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
    cost_H += CoM_weight * J_com_copy.transpose() * J_com_copy;
    cost_F += CoM_weight * J_com_copy.transpose() * (state_desired.com_vel + CoM_gains*com_pos_error);

    cost_H += left_weight * J_left_foot.transpose() * J_left_foot;
    cost_F += left_weight * J_left_foot.transpose() * (state_desired.left_foot_vel + left_weight*left_foot_error);

    cost_H += right_weight * J_right_foot.transpose() * J_right_foot;
    cost_F += right_weight * J_right_foot.transpose() * (state_desired.right_foot_vel + right_weight*right_foot_error);

    // Add the constraints
    Eigen::MatrixXd constrained_joints = 0 * Eigen::MatrixXd::Identity(m_model.nv, m_model.nv);
    Eigen::VectorXd joint_upper_lim = 0 * 10 * Eigen::VectorXd::Ones(m_model.nv);
    Eigen::VectorXd joint_lower_lim = -0 * 10 * Eigen::VectorXd::Ones(m_model.nv);

    // dummy equality constraint
    // This is needed to set one of the feet to a fixed position
    // Equal to structure A*q_dot = b
    // We set these to zero so we know we cannot influence this q_dot
    Eigen::MatrixXd A_dummy = Eigen::MatrixXd::Zero(6, m_model.nv);
    Eigen::VectorXd b_dummy = Eigen::VectorXd::Zero(6);

    // We now swap the dummy foot based on the fixed Jacobian.
    if (stance_foot==1)
        {
            A_dummy = J_left_foot;
        }
    else
        {
            A_dummy = J_right_foot;
        }
    // if (walkState.supportFoot == Foot::LEFT) A_dummy = Jacobian_leftFoot;
    // else				     A_dummy = Jacobian_rightFoot;

    m_qp_solver_ptr->solve(cost_H, cost_F, A_dummy, b_dummy, constrained_joints, joint_lower_lim, joint_upper_lim);
    Eigen::VectorXd velocity_trajectory = m_qp_solver_ptr->get_solution();

    return velocity_trajectory.tail(8); // CHANGE 50 to [DOF-6], so [14-6]
}

Eigen::VectorXd IkSolver::velocity_to_pos(Eigen::VectorXd& velocity, double dt)
{
    Eigen::VectorXd new_position;
    // We assume that the velocity vector is ordered the same way as the joint vector
    for(int i=0 ; i<m_model.joints.size(); i++)
        {
        new_position[i] = m_model.joints[i].idx_q() + dt*velocity[i];
        }
    return new_position;
}

const pinocchio::Model IkSolver::get_model()
{
    return m_model;
}

void IkSolver::set_current_state()
{
    pinocchio::forwardKinematics(m_model, m_model_data, m_joint_pos);
    pinocchio::updateFramePlacements(m_model, m_model_data);
    pinocchio::FrameIndex left_foot_index = m_model.getFrameId("foot_left");
    pinocchio::FrameIndex right_foot_index = m_model.getFrameId("foot_left");

    m_current_state.left_foot_pose << pinocchio::rpy::matrixToRpy(m_model_data.oMf[left_foot_index].rotation()),
        m_model_data.oMf[left_foot_index].translation();
    m_current_state.right_foot_pose << pinocchio::rpy::matrixToRpy(m_model_data.oMf[right_foot_index].rotation()),
        m_model_data.oMf[right_foot_index].translation();
    // m_current_state.com_pos << pinocchio::centerOfMass(m_model, m_model_data, m_joint_pos);
    // m_current_state.left_foot_pose <<
    // pinocchio::rpy::matrixToRpy(m_model.frames[left_foot_index].placement.rotation()),
    //     m_model.frames[left_foot_index].placement.translation();
    // m_current_state.right_foot_pose << pinocchio::rpy::matrixToRpy(
    //     m_model.frames[right_foot_index].placement.rotation()),
    //     m_model.frames[right_foot_index].placement.translation();
}

const state IkSolver::get_state()
{
    return m_current_state;
}