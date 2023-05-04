//
// Created by Sahand March 6th 2023
//
#include "ik_solver/ik_solver.hpp"
#include "rclcpp/rclcpp.hpp"

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
    m_joint_vel.resize(m_model.nv);
    pinocchio::forwardKinematics(m_model, m_model_data, m_joint_pos);
    pinocchio::updateFramePlacements(m_model, m_model_data);
}

void IkSolver::set_joint_configuration(sensor_msgs::msg::JointState::SharedPtr msg)
{
    // We also update our map here
    for (long unsigned int i = 0; i < msg->name.size(); i++) {
        pinocchio::JointIndex index = m_model.getJointId(msg->name[i]);
        // RCLCPP_INFO(rclcpp::get_logger(""), "joint pos size %i", m_joint_pos.size());
        // RCLCPP_INFO(rclcpp::get_logger(""), "Added key with name %s, index %i, special index %i", msg->name[i].c_str(), i, index);
        if (index-1<m_joint_pos.size()){
            m_joint_pos[index-1] = msg->position[i];
            m_joint_vel[index-1] = msg->velocity[i];;
            m_pinocchio_to_march_joint_map.insert(std::pair<int,int>(index, i));
            // RCLCPP_INFO(rclcpp::get_logger(""), "(%s, %i)", msg->name[i].c_str(), index-1);
        }
        // m_model.joints[index].setIndexes(index, msg->position[i], msg->velocity[i]);
    }
}

int IkSolver::set_jacobian()
{
    try { 
        pinocchio::JointIndex left_foot_index = m_model.getFrameId("L_foot");
        pinocchio::FrameIndex right_foot_index = m_model.getFrameId("R_foot");
        pinocchio::getJointJacobian(
            m_model, m_model_data, m_model.frames[left_foot_index].parent, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_left_foot);
        pinocchio::getJointJacobian(
            m_model, m_model_data, m_model.frames[right_foot_index].parent, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_right_foot);
        // pinocchio::getFrameJacobian(m_model, m_model_data, m_model.frames[left_foot_index].parent, pinocchio::ReferenceFrame::LOCAL, J_left_foot);
        // pinocchio::getFrameJacobian(m_model, m_model_data, m_model.frames[right_foot_index].parent, pinocchio::ReferenceFrame::LOCAL, J_right_foot);

        // Print the cost vectors
        std::stringstream ss;
        ss << J_left_foot.format(Eigen::IOFormat(6, m_model.nv, ", ", "\n", "", ""));
        // RCLCPP_INFO(rclcpp::get_logger(""), "Left foot Jacobian is :\n" + ss.str() + "\n");
        ss.clear();
        ss.str("");

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

Eigen::VectorXd IkSolver::solve_for_velocity(state state_current, state state_desired, const double dt, int stance_foot)
{
    // We put the weights here, but of course we can remove them later
    // WEIGHTS
    double left_weight = 1;
    double right_weight = 0.0;
    double CoM_weight = 0.0;
    double qdot_weight = 1e-6;
    // double base_weight = 1;

    double CoM_gains = 0.0/dt;
    double left_gains = 1/dt;
    double right_gains = 1/dt;
    // RCLCPP_INFO(rclcpp::get_logger(""), "Initialized all weights and gains");

    // Get the error vectors
    // Here, we assume the jacobians have already been set
    // The error is equal to the desired state, as we express them in local coordinates
    Eigen::VectorXd left_foot_error = state_desired.left_foot_pose;
    left_foot_error.segment(3, 3)
        = angleSignedDistance(state_desired.left_foot_pose.segment(3, 3), state_current.left_foot_pose.segment(3, 3));
    Eigen::VectorXd right_foot_error = state_desired.right_foot_pose;
    right_foot_error.segment(3, 3)
        = angleSignedDistance(state_desired.left_foot_pose.segment(3, 3), state_current.left_foot_pose.segment(3, 3));
    Eigen::VectorXd com_pos_error = state_desired.com_pos;


    // Print the error vectors
    // ss << left_foot_error.format(Eigen::IOFormat(6, 0, ", ", "\n", "", ""));
    // RCLCPP_INFO(rclcpp::get_logger(""), "Left foot error is :\n" + ss.str() + "\n");
    // ss.clear();
    // ss.str("");
    // ss << right_foot_error.format(Eigen::IOFormat(6, 0, ", ", "\n", "", ""));
    // RCLCPP_INFO(rclcpp::get_logger(""), "Right foot error is :\n" + ss.str() + "\n");
    // ss.clear();
    // ss.str("");
    // RCLCPP_INFO(rclcpp::get_logger(""), "Initialized all error vectors");
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

    // RCLCPP_INFO(rclcpp::get_logger(""), "Initialized the empty cost function");

    // NOTE::: CONSIDER ADDING THE SINGULARITY ROBUST REGULARIZATION TERM
    // [NOTE]: ADD THE ILNEAR TERM USING COM VELOCITIES
    cost_H += left_weight * (J_left_foot.transpose() * J_left_foot + Eigen::MatrixXd::Identity(m_model.nv, m_model.nv)*0.001);
    // RCLCPP_INFO(rclcpp::get_logger(""), "Added the quadratic left_foot cost");
    cost_F += -left_weight * J_left_foot.transpose() * (state_desired.left_foot_vel + left_gains * left_foot_error);
    // RCLCPP_INFO(rclcpp::get_logger(""), "Added the linear left_foot cost");


    cost_H += right_weight * (J_right_foot.transpose() * J_right_foot + Eigen::MatrixXd::Identity(m_model.nv, m_model.nv)*0.001);
    // RCLCPP_INFO(rclcpp::get_logger(""), "Added the quadratic right_foot cost");
    cost_F += -right_weight * J_right_foot.transpose() * (state_desired.right_foot_vel + right_gains * right_foot_error);
    // RCLCPP_INFO(rclcpp::get_logger(""), "Added the linear right_foot cost");
    // 
    cost_H += CoM_weight * (J_com.transpose() * J_com + Eigen::MatrixXd::Identity(m_model.nv, m_model.nv)*0.001);
    // RCLCPP_INFO(rclcpp::get_logger(""), "Added the quadratic com cost");
    cost_F += -CoM_weight * J_com.transpose() * (state_desired.com_vel + CoM_gains * com_pos_error);
    // RCLCPP_INFO(rclcpp::get_logger(""), "Added the linear com cost");

    // RCLCPP_INFO(rclcpp::get_logger(""), "Initialized cost function");

    // Add the constraints
    Eigen::MatrixXd constrained_joints = 0 * Eigen::MatrixXd::Identity(m_model.nv, m_model.nv);
    Eigen::VectorXd joint_upper_lim = 0 * 10 * Eigen::VectorXd::Ones(m_model.nv);
    Eigen::VectorXd joint_lower_lim = -0 * 10 * Eigen::VectorXd::Ones(m_model.nv);

    // RCLCPP_INFO(rclcpp::get_logger(""), "Initialized constraints");

    // dummy equality constraint
    // This is needed to set one of the feet to a fixed position
    // Equal to structure A*q_dot = b
    // We set these to zero so we know we cannot influence this q_dot
    Eigen::MatrixXd A_dummy = Eigen::MatrixXd::Zero(6, m_model.nv);
    Eigen::VectorXd b_dummy = Eigen::VectorXd::Zero(6);

    // RCLCPP_INFO(rclcpp::get_logger(""), "Initialized dummy equality constraints");

    // We now swap the dummy foot based on the fixed Jacobian.
    if (stance_foot == -1) {
        A_dummy = J_left_foot;
    } else {
        A_dummy = J_right_foot;
    }

    // RCLCPP_INFO(rclcpp::get_logger(""), "Initialized dummy foot");
    // if (walkState.supportFoot == Foot::LEFT) A_dummy = Jacobian_leftFoot;
    // else				     A_dummy = Jacobian_rightFoot;

    m_qp_solver_ptr->solve(cost_H, cost_F, A_dummy, b_dummy, constrained_joints, joint_lower_lim, joint_upper_lim);
    Eigen::VectorXd velocity_trajectory = m_qp_solver_ptr->get_solution();

    return velocity_trajectory; // CHANGE 50 to [DOF-6], so [14-6]
}

Eigen::VectorXd IkSolver::velocity_to_pos(Eigen::VectorXd& velocity, const double dt)
{
    Eigen::VectorXd new_position = Eigen::VectorXd::Zero(velocity.size());
    std::map<int, int>::iterator it;

    for (auto i : m_model.joints) {
        it = m_pinocchio_to_march_joint_map.find(i.id_impl());
        // RCLCPP_INFO(rclcpp::get_logger("ik_solver"), "Looking for Joint:  %i", i.id_impl());
        if (it!=m_pinocchio_to_march_joint_map.end())
            {
            // RCLCPP_INFO(rclcpp::get_logger("ik_solver"), "found key! with index (%i) and angle %f", it->second,m_joint_pos[it->second]);
            new_position[it->second] = m_joint_pos[i.id_impl()-1] + dt * velocity[i.id_impl()-1];
            // RCLCPP_INFO(rclcpp::get_logger("ik_solver"), "new position of joint [%i] is %f + (%f * %f) = %f", it->second,m_joint_pos[it->second], dt, velocity[it->second], new_position[it->second]);
            }
    }
    return new_position;
}

const pinocchio::Model IkSolver::get_model()
{
    return m_model;
}

void IkSolver::set_current_state()
{
    pinocchio::forwardKinematics(m_model, m_model_data, m_joint_pos, m_joint_vel);
    pinocchio::computeJointJacobians(m_model, m_model_data, m_joint_pos);
    pinocchio::updateFramePlacements(m_model, m_model_data);
    pinocchio::FrameIndex left_foot_index = m_model.getFrameId("L_foot");
    pinocchio::FrameIndex right_foot_index = m_model.getFrameId("R_foot");

    m_current_state.left_foot_pose << pinocchio::rpy::matrixToRpy(m_model_data.oMf[left_foot_index].rotation()),
        m_model_data.oMf[left_foot_index].translation();
    m_current_state.right_foot_pose << pinocchio::rpy::matrixToRpy(m_model_data.oMf[right_foot_index].rotation()),
        m_model_data.oMf[right_foot_index].translation();

    m_current_state.com_pos << pinocchio::centerOfMass(m_model, m_model_data, m_joint_pos), 0.0, 0.0, 0.0;
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