//
// Created by Tessel & Jack March 6th 2023
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
    pinocchio::forwardKinematics(m_model,m_model_data,m_joint_pos);
    pinocchio::updateFramePlacements(m_model,m_model_data);
    pinocchio::FrameIndex test_foot_index = m_model.getFrameId("foot_left");
}

void IkSolver::set_joint_configuration()
{
    m_joint_pos = pinocchio::randomConfiguration(m_model);
    // We apply forward kinematics to get the cartesian positions
    pinocchio::forwardKinematics(m_model,m_model_data,m_joint_pos);

}

int IkSolver::set_jacobian()
{
    try {
        pinocchio::FrameIndex left_foot_index = m_model.getFrameId("foot_left");
        pinocchio::FrameIndex right_foot_index = m_model.getFrameId("foot_left");
        pinocchio::computeJointJacobian(m_model, m_model_data, m_joint_pos, m_model.frames[left_foot_index].parent, J_left_foot);
        pinocchio::computeJointJacobian(m_model, m_model_data, m_joint_pos, m_model.frames[right_foot_index].parent, J_right_foot);
        pinocchio::Data::Matrix3x angular_com_matrix(3, m_model.nv);
        angular_com_matrix.setZero();
        J_com << pinocchio::jacobianCenterOfMass(m_model, m_model_data, m_joint_pos),angular_com_matrix;
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

double IkSolver::get_angle_difference(double theta_des, double theta)
    {
    double a_dif=0;
    Eigen::Vector2d n_des,n;

    n_des<<cos(theta_des), sin(theta_des);
    n<<cos(theta), sin(theta);

    double alpha_absolute = acos(n_des.transpose()*n);

    Eigen::Vector3d n3,nD3;

    n3<<n(0),n(1),0;
    nD3<<n_des(0),n_des(1),0;

    Eigen::Vector3d nC3;

    nC3=n3.cross(nD3);

    if(nC3(2)>0){
        a_dif=alpha_absolute;
    }
    else{
        a_dif=-alpha_absolute;
    }

    return a_dif;
    }

Eigen::VectorXd IkSolver::solve_for_velocity(state state_current, state state_desired)
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
    

    // Get the error difference compared to the 
    // Here, we assume the jacobians have already been set

    return Eigen::VectorXd(6);
}

const pinocchio::Model IkSolver::get_model()
    {
        return m_model;
    }

void IkSolver::set_current_state()
    {
    pinocchio::forwardKinematics(m_model,m_model_data,m_joint_pos);
    pinocchio::updateFramePlacements(m_model,m_model_data);
    pinocchio::FrameIndex left_foot_index = m_model.getFrameId("foot_left");
    pinocchio::FrameIndex right_foot_index = m_model.getFrameId("foot_left");
    m_current_state.left_foot_pos << m_model.frames[left_foot_index].placement.translation(); 
    }