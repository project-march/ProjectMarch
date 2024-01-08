#include "state_estimator/exo_estimator.hpp"

// #include <functional>
// #include <memory>
// #include <algorithm>
#include <chrono>
#include "math.h"

ExoEstimator::ExoEstimator()
{
    // Initialize model. TODO: Load from YAML file
    urdf_path_ = ament_index_cpp::get_package_share_directory("march_description") 
        + "/urdf/march8/hennie_with_koen.urdf";

    // Initialize KDL chain
    root_link_ = "backpack";
    left_foot_link_ = "L_foot";
    right_foot_link_ = "R_foot";
    if (!kdl_parser::treeFromFile(urdf_path_, kdl_tree_))
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to construct KDL tree");
    }
    if (!kdl_tree_.getChain(root_link_, left_foot_link_, kdl_chain_left_foot_))
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to construct KDL chain to left foot");
    }
    if (!kdl_tree_.getChain(root_link_, right_foot_link_, kdl_chain_right_foot_))
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to construct KDL chain to right foot");
    }

    // Print the KDL chain
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Number of joints in KDL chain to left foot: %d", kdl_chain_left_foot_.getNrOfJoints());
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Number of joints in KDL chain to right foot: %d", kdl_chain_right_foot_.getNrOfJoints());
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "KDL chain to left foot:");
    for (unsigned int i = 0; i < kdl_chain_left_foot_.getNrOfSegments(); i++)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Segment %d: %s", i, kdl_chain_left_foot_.getSegment(i).getName().c_str());
    }
    for (unsigned int i = 0; i < kdl_chain_left_foot_.getNrOfJoints(); i++)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Joint %d: %s", i, kdl_chain_left_foot_.getSegment(i).getJoint().getName().c_str());
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "KDL chain to right foot:");
    for (unsigned int i = 0; i < kdl_chain_right_foot_.getNrOfSegments(); i++)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Segment %d: %s", i, kdl_chain_right_foot_.getSegment(i).getName().c_str());
    }
    for (unsigned int i = 0; i < kdl_chain_right_foot_.getNrOfJoints(); i++)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Joint %d: %s", i, kdl_chain_right_foot_.getSegment(i).getJoint().getName().c_str());
    }

    q_ = Eigen::VectorXd::Zero(8);
    joint_positions_reset_ = true;
}

// ExoEstimator::~ExoEstimator()
// {

// }

void ExoEstimator::initialize_model()
{
    pinocchio::urdf::buildModel(urdf_path_, model_);
    data_ = pinocchio::Data(model_);
}

void ExoEstimator::setJointPositions(std::vector<double> joint_positions)
{
    if (joint_positions_reset_)
    {
        Eigen::VectorXd q = Eigen::VectorXd::Map(joint_positions.data(), joint_positions.size());
        q_(0) = q(3);
        q_(1) = q(0);
        q_(2) = q(1);
        q_(3) = q(2);
        q_(4) = q(7);
        q_(5) = q(4);
        q_(6) = q(5);
        q_(7) = q(6);
        joint_positions_reset_ = false;
    }
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Joint positions set");
    // for (int i = 0; i < q_.size(); i++)
    // {
    //     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Joint %d: %f", i, q_(i));
    // }
}

void ExoEstimator::resetJointPositions()
{
    joint_positions_reset_ = true;
}

// std::vector<double> ExoEstimator::getFeetPositions()
// {
//     std::vector<double> feet_positions;

//     KDL::ChainFkSolverPos_recursive kdl_solver_left_foot_ = KDL::ChainFkSolverPos_recursive(kdl_chain_left_foot_);
//     KDL::ChainFkSolverPos_recursive kdl_solver_right_foot_ = KDL::ChainFkSolverPos_recursive(kdl_chain_right_foot_);
//     KDL::JntArray kdl_q_left_foot = KDL::JntArray(kdl_chain_left_foot_.getNrOfJoints());
//     KDL::JntArray kdl_q_right_foot = KDL::JntArray(kdl_chain_right_foot_.getNrOfJoints());
//     for (int i = 0; i < 4; i++)
//     {
//         kdl_q_left_foot(i) = q_(i);
//         kdl_q_right_foot(i) = q_(4+i);
//     }
//     KDL::Frame kdl_left_foot_frame = KDL::Frame();
//     KDL::Frame kdl_right_foot_frame = KDL::Frame();
//     kdl_solver_left_foot_.JntToCart(kdl_q_left_foot, kdl_left_foot_frame);
//     kdl_solver_right_foot_.JntToCart(kdl_q_right_foot, kdl_right_foot_frame);

//     for (int i = 0; i < 3; i++)
//     {
//         feet_positions.push_back(kdl_left_foot_frame.p(i));
//     }
//     for (int i = 0; i < 3; i++)
//     {
//         feet_positions.push_back(kdl_right_foot_frame.p(i));
//     }

//     return feet_positions;
// }
std::vector<double> ExoEstimator::getFeetPositions()
{
    std::vector<double> feet_positions;

    double q_LHAA = q_(0);
    double q_LHFE = q_(1);
    double q_LKFE = q_(2);
    double q_LADPF = q_(3);
    double q_RHAA = q_(4);
    double q_RHFE = q_(5);
    double q_RKFE = q_(6);
    double q_RADPF = q_(7);

    Eigen::VectorXd x = Eigen::VectorXd::Zero(6);
    // x(0) = 0.41*sin(q_LHFE - q_LKFE)*cos(q_LHAA) + 0.0562224*sin(q_LADPF + q_LHFE - q_LKFE)*cos(q_LHAA) + 0.4*sin(q_LHFE)*cos(q_LHAA) + 0.0562224*cos(q_LADPF + q_LHFE - q_LKFE) + 0.014;
    // x(1) = 0.8662224*sin(q_LHAA) + 0.16;
    // x(2) = 0.0562224*sin(q_LADPF + q_LHFE - q_LKFE) - 0.41*cos(q_LHFE - q_LKFE)*cos(q_LHAA) - 0.0562224*cos(q_LADPF + q_LHFE - q_LKFE)*cos(q_LHAA) - 0.4*cos(q_LHAA)*cos(q_LHFE) - 0.144;
    // x(3) = 0.41*sin(q_RHFE - q_RKFE)*cos(q_RHAA) + 0.0562224*sin(q_RADPF + q_RHFE - q_RKFE)*cos(q_RHAA) + 0.4*sin(q_RHFE)*cos(q_RHAA) + 0.0562224*cos(q_RADPF + q_RHFE - q_RKFE) + 0.014;
    // x(4) = 0.8662224*sin(q_RHAA) - 0.16;
    // x(5) = 0.0562224*sin(q_RADPF + q_RHFE - q_RKFE) - 0.41*cos(q_RHFE - q_RKFE)*cos(q_RHAA) - 0.0562224*cos(q_RADPF + q_RHFE - q_RKFE)*cos(q_RHAA) - 0.4*cos(q_RHAA)*cos(q_RHFE) - 0.144;
    x(0) = 0.41*sin(q_LHFE - q_LKFE)*cos(q_LHAA) + 0.4*sin(q_LHFE)*cos(q_LHAA) + 0.014;
    x(1) = 0.81*sin(q_LHAA) + 0.16;
    x(2) = -0.41*cos(q_LHFE - q_LKFE)*cos(q_LHAA) - 0.4*cos(q_LHAA)*cos(q_LHFE) - 0.144;
    x(3) = 0.41*sin(q_RHFE - q_RKFE)*cos(q_RHAA) + 0.4*sin(q_RHFE)*cos(q_RHAA) + 0.014;
    x(4) = 0.81*sin(q_RHAA) - 0.16;
    x(5) = -0.41*cos(q_RHFE - q_RKFE)*cos(q_RHAA) - 0.4*cos(q_RHAA)*cos(q_RHFE) - 0.144;

    feet_positions = std::vector<double>(x.data(), x.data() + x.size());

    return feet_positions;
}

// std::vector<double> ExoEstimator::getJacobian()
// {
//     std::vector<double> jacobian;

//     // Calculating Jacobians using KDL
//     // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Calculating Jacobians using KDL...");
//     KDL::ChainJntToJacSolver kdl_solver_left_foot_ = KDL::ChainJntToJacSolver(kdl_chain_left_foot_);
//     KDL::ChainJntToJacSolver kdl_solver_right_foot_ = KDL::ChainJntToJacSolver(kdl_chain_right_foot_);
//     // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "KDL solvers initialized");
//     KDL::JntArray kdl_q_left_foot = KDL::JntArray(kdl_chain_left_foot_.getNrOfJoints());
//     KDL::JntArray kdl_q_right_foot = KDL::JntArray(kdl_chain_right_foot_.getNrOfJoints());
//     // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "KDL joint arrays initialized");
//     for (int i = 0; i < 4; i++)
//     {
//         kdl_q_left_foot(i) = q_(i);
//         kdl_q_right_foot(i) = q_(4+i);
//     }
//     // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Left foot joint array: %f, %f, %f, %f", kdl_q_left_foot(0), kdl_q_left_foot(1), kdl_q_left_foot(2), kdl_q_left_foot(3));
//     // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Right foot joint array: %f, %f, %f, %f", kdl_q_right_foot(0), kdl_q_right_foot(1), kdl_q_right_foot(2), kdl_q_right_foot(3));
//     // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "KDL joint arrays filled");
//     KDL::Jacobian kdl_jacobian_left_foot = KDL::Jacobian(kdl_chain_left_foot_.getNrOfJoints());
//     KDL::Jacobian kdl_jacobian_right_foot = KDL::Jacobian(kdl_chain_right_foot_.getNrOfJoints());
//     kdl_jacobian_left_foot.data.setZero();
//     kdl_jacobian_right_foot.data.setZero();
//     // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "KDL Jacobians initialized");
//     kdl_solver_left_foot_.JntToJac(kdl_q_left_foot, kdl_jacobian_left_foot);
//     kdl_solver_right_foot_.JntToJac(kdl_q_right_foot, kdl_jacobian_right_foot);
//     // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "KDL Jacobians calculated");

//     // for (int i = 0; i < 6; i++)
//     // {
//     //     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Left foot Jacobian row %d: %f, %f, %f, %f", i, kdl_jacobian_left_foot.data(i, 0), kdl_jacobian_left_foot.data(i, 1), kdl_jacobian_left_foot.data(i, 2), kdl_jacobian_left_foot.data(i, 3));
//     // }
//     // for (int i = 0; i < 6; i++)
//     // {
//     //     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Right foot Jacobian row %d: %f, %f, %f, %f", i, kdl_jacobian_right_foot.data(i, 0), kdl_jacobian_right_foot.data(i, 1), kdl_jacobian_right_foot.data(i, 2), kdl_jacobian_right_foot.data(i, 3));
//     // }

//     for (int j = 0; j < 4; j++)
//     {
//         for (int i = 0; i < 3; i++)
//         {
//             // jacobian.push_back(J_left_foot(i, j));
//             jacobian.push_back(kdl_jacobian_left_foot.data(i, j));
//         }
//         for (int i = 0; i < 3; i++)
//         {
//             jacobian.push_back(0.0);
//         }
//     }
//     for (int j = 0; j < 4; j++)
//     {
//         for (int i = 0; i < 3; i++)
//         {
//             jacobian.push_back(0.0);
//         }
//         for (int i = 0; i < 3; i++)
//         {
//             // jacobian.push_back(J_right_foot(i, j));
//             jacobian.push_back(kdl_jacobian_right_foot.data(i, j));
//         }
//     }
//     // jacobian = std::vector<double>(kdl_jacobian_left_foot.data.data(), kdl_jacobian_left_foot.data.data() + kdl_jacobian_left_foot.data.rows() * kdl_jacobian_left_foot.data.cols());

//     Eigen::MatrixXd jacobianMatrix = Eigen::MatrixXd::Zero(6, 8);
//     jacobianMatrix = Eigen::Map<Eigen::MatrixXd>(jacobian.data(), 6, 8);
//     // for (int i = 0; i < 6; i++)
//     // {
//     //     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Jacobian row %d: %f, %f, %f, %f, %f, %f, %f, %f", i, jacobianMatrix(i, 0), jacobianMatrix(i, 1), jacobianMatrix(i, 2), jacobianMatrix(i, 3), jacobianMatrix(i, 4), jacobianMatrix(i, 5), jacobianMatrix(i, 6), jacobianMatrix(i, 7));
//     // }

//     return jacobian;
// }
std::vector<double> ExoEstimator::getJacobian()
{
    std::vector<double> jacobian;

    // double q_LHAA = -q_(0);
    // double q_LHFE = -q_(1);
    // double q_LKFE = q_(2);
    // double q_LADPF = -q_(3);
    // double q_RHAA = q_(4);
    // double q_RHFE = -q_(5);
    // double q_RKFE = q_(6);
    // double q_RADPF = -q_(7);
    double q_LHAA = q_(0);
    double q_LHFE = q_(1);
    double q_LKFE = q_(2);
    double q_LADPF = q_(3);
    double q_RHAA = q_(4);
    double q_RHFE = q_(5);
    double q_RKFE = q_(6);
    double q_RADPF = q_(7);

    Eigen::MatrixXd jacobianMatrix = Eigen::MatrixXd::Zero(6, 8);

    // // Left foot Jacobian.
    // jacobianMatrix(0,0) = -0.41*sin(q_LHFE - q_LKFE)*sin(q_LHAA) - 0.0562224*sin(q_LADPF + q_LHFE - q_LKFE)*sin(q_LHAA) - 0.4*sin(q_LHAA)*sin(q_LHFE);
    // jacobianMatrix(0,1) = -0.0562224*sin(q_LADPF + q_LHFE - q_LKFE) + 0.41*cos(q_LHFE - q_LKFE)*cos(q_LHAA) + 0.0562224*cos(q_LADPF + q_LHFE - q_LKFE)*cos(q_LHAA) + 0.4*cos(q_LHAA)*cos(q_LHFE);
    // jacobianMatrix(0,2) = 0.0562224*sin(q_LADPF + q_LHFE - q_LKFE) - 0.41*cos(q_LHFE - q_LKFE)*cos(q_LHAA) - 0.0562224*cos(q_LADPF + q_LHFE - q_LKFE)*cos(q_LHAA);
    // jacobianMatrix(0,3) = -0.0562224*sin(q_LADPF + q_LHFE - q_LKFE) + 0.0562224*cos(q_LADPF + q_LHFE - q_LKFE)*cos(q_LHAA);

    // jacobianMatrix(1,0) = 0.8662224*cos(q_LHAA);

    // jacobianMatrix(2,0) = 0.41*sin(q_LHAA)*cos(q_LHFE - q_LKFE) + 0.0562224*sin(q_LHAA)*cos(q_LADPF + q_LHFE - q_LKFE) + 0.4*sin(q_LHAA)*cos(q_LHFE);
    // jacobianMatrix(2,1) = 0.41*sin(q_LHFE - q_LKFE)*cos(q_LHAA) + 0.0562224*sin(q_LADPF + q_LHFE - q_LKFE)*cos(q_LHAA) + 0.4*sin(q_LHFE)*cos(q_LHAA) + 0.0562224*cos(q_LADPF + q_LHFE - q_LKFE);
    // jacobianMatrix(2,2) = -0.41*sin(q_LHFE - q_LKFE)*cos(q_LHAA) - 0.0562224*sin(q_LADPF + q_LHFE - q_LKFE)*cos(q_LHAA) - 0.0562224*cos(q_LADPF + q_LHFE - q_LKFE);
    // jacobianMatrix(2,3) = 0.0562224*sin(q_LADPF + q_LHFE - q_LKFE)*cos(q_LHAA) + 0.0562224*cos(q_LADPF + q_LHFE - q_LKFE);

    // // Right foot Jacobian.
    // jacobianMatrix(3,4) = -0.41*sin(q_RHFE - q_RKFE)*sin(q_RHAA) - 0.0562224*sin(q_RADPF + q_RHFE - q_RKFE)*sin(q_RHAA) - 0.4*sin(q_RHAA)*sin(q_RHFE);
    // jacobianMatrix(3,5) = -0.0562224*sin(q_RADPF + q_RHFE - q_RKFE) + 0.41*cos(q_RHFE - q_RKFE)*cos(q_RHAA) + 0.0562224*cos(q_RADPF + q_RHFE - q_RKFE)*cos(q_RHAA) + 0.4*cos(q_RHAA)*cos(q_RHFE);
    // jacobianMatrix(3,6) = 0.0562224*sin(q_RADPF + q_RHFE - q_RKFE) - 0.41*cos(q_RHFE - q_RKFE)*cos(q_RHAA) - 0.0562224*cos(q_RADPF + q_RHFE - q_RKFE)*cos(q_RHAA);
    // jacobianMatrix(3,7) = -0.0562224*sin(q_RADPF + q_RHFE - q_RKFE) + 0.0562224*cos(q_RADPF + q_RHFE - q_RKFE)*cos(q_RHAA);

    // jacobianMatrix(4,4) = 0.8662224*cos(q_RHAA);

    // jacobianMatrix(5,4) = 0.41*sin(q_RHAA)*cos(q_RHFE - q_RKFE) + 0.0562224*sin(q_RHAA)*cos(q_RADPF + q_RHFE - q_RKFE) + 0.4*sin(q_RHAA)*cos(q_RHFE);
    // jacobianMatrix(5,5) = 0.41*sin(q_RHFE - q_RKFE)*cos(q_RHAA) + 0.0562224*sin(q_RADPF + q_RHFE - q_RKFE)*cos(q_RHAA) + 0.4*sin(q_RHFE)*cos(q_RHAA) + 0.0562224*cos(q_RADPF + q_RHFE - q_RKFE);
    // jacobianMatrix(5,6) = -0.41*sin(q_RHFE - q_RKFE)*cos(q_RHAA) - 0.0562224*sin(q_RADPF + q_RHFE - q_RKFE)*cos(q_RHAA) - 0.0562224*cos(q_RADPF + q_RHFE - q_RKFE);
    // jacobianMatrix(5,7) = 0.0562224*sin(q_RADPF + q_RHFE - q_RKFE)*cos(q_RHAA) + 0.0562224*cos(q_RADPF + q_RHFE - q_RKFE);

    // Left foot Jacobian.
    jacobianMatrix(0,0) = -0.41*sin(q_LHFE - q_LKFE)*sin(q_LHAA) - 0.4*sin(q_LHAA)*sin(q_LHFE);
    jacobianMatrix(0,1) = 0.41*cos(q_LHFE - q_LKFE)*cos(q_LHAA) + 0.4*cos(q_LHAA)*cos(q_LHFE);
    jacobianMatrix(0,2) = -0.41*cos(q_LHFE - q_LKFE)*cos(q_LHAA);

    jacobianMatrix(1,0) = 0.81*cos(q_LHAA);

    jacobianMatrix(2,0) = 0.41*sin(q_LHAA)*cos(q_LHFE - q_LKFE) + 0.4*sin(q_LHAA)*cos(q_LHFE);
    jacobianMatrix(2,1) = 0.41*sin(q_LHFE - q_LKFE)*cos(q_LHAA) + 0.4*sin(q_LHFE)*cos(q_LHAA);
    jacobianMatrix(2,2) = -0.41*sin(q_LHFE - q_LKFE)*cos(q_LHAA);

    // Right foot Jacobian.
    jacobianMatrix(3,4) = -0.41*sin(q_RHFE - q_RKFE)*sin(q_RHAA) - 0.4*sin(q_RHAA)*sin(q_RHFE);
    jacobianMatrix(3,5) = 0.41*cos(q_RHFE - q_RKFE)*cos(q_RHAA) + 0.4*cos(q_RHAA)*cos(q_RHFE);
    jacobianMatrix(3,6) = -0.41*cos(q_RHFE - q_RKFE)*cos(q_RHAA);

    jacobianMatrix(4,4) = 0.81*cos(q_RHAA);

    jacobianMatrix(5,4) = .41*sin(q_RHAA)*cos(q_RHFE - q_RKFE) + 0.4*sin(q_RHAA)*cos(q_RHFE);
    jacobianMatrix(5,6) = .41*sin(q_RHFE - q_RKFE)*cos(q_RHAA) + 0.4*sin(q_RHFE)*cos(q_RHAA);
    jacobianMatrix(5,7) = -0.41*sin(q_RHFE - q_RKFE)*cos(q_RHAA);

    // for (int i = 0; i < 6; i++)
    // {
    //     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Jacobian row %d: %f, %f, %f, %f, %f, %f, %f, %f", i, jacobianMatrix(i, 0), jacobianMatrix(i, 1), jacobianMatrix(i, 2), jacobianMatrix(i, 3), jacobianMatrix(i, 4), jacobianMatrix(i, 5), jacobianMatrix(i, 6), jacobianMatrix(i, 7));
    // }

    jacobian = std::vector<double>(jacobianMatrix.data(), jacobianMatrix.data() + jacobianMatrix.rows() * jacobianMatrix.cols());

    return jacobian;
}