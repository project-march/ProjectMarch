#include "state_estimator/exo_estimator.hpp"

// #include <functional>
// #include <memory>
// #include <algorithm>
#include <chrono>

ExoEstimator::ExoEstimator()
{
    // Initialize model. TODO: Load from YAML file
    urdf_path_ = ament_index_cpp::get_package_share_directory("march_description") 
        + "/urdf/march8/hennie_with_koen.urdf";
    initialize_model();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ExoEstimator initialized");
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "URDF path: %s", urdf_path_.c_str());
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Model name: %s", model_.name.c_str());
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Model nq: %d", model_.nq);
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Model nv: %d", model_.nv);
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "LHAA: %d", model_.getJointId("left_hip_aa"));
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "LHFE: %d", model_.getJointId("left_hip_fe"));
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "LKFE: %d", model_.getJointId("left_knee"));
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "LADPF: %d", model_.getJointId("left_ankle"));
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "RHAA: %d", model_.getJointId("right_hip_aa"));
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "RHFE: %d", model_.getJointId("right_hip_fe"));
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "RKFE: %d", model_.getJointId("right_knee"));
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "RADPF: %d", model_.getJointId("right_ankle"));
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Model njoints: %d", model_.njoints);
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Model nframes: %d", model_.nframes);

    // q_ = Eigen::VectorXd::Zero(model_.nq);
    q_ = pinocchio::neutral(model_);

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
    for (int i = 0; i < kdl_chain_left_foot_.getNrOfSegments(); i++)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Segment %d: %s", i, kdl_chain_left_foot_.getSegment(i).getName().c_str());
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "KDL chain to right foot:");
    for (int i = 0; i < kdl_chain_right_foot_.getNrOfSegments(); i++)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Segment %d: %s", i, kdl_chain_right_foot_.getSegment(i).getName().c_str());
    }
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
    q_ = Eigen::VectorXd::Map(joint_positions.data(), joint_positions.size());
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Joint positions set");
    // for (int i = 0; i < q_.size(); i++)
    // {
    //     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Joint %d: %f", i, q_(i));
    // }
}

std::vector<double> ExoEstimator::getFeetPositions()
{
    std::vector<double> feet_positions;

    pinocchio::forwardKinematics(model_, data_, q_);
    pinocchio::updateFramePlacements(model_, data_);

    // pinocchio::SE3 T_left_foot_backpack = pinocchio::updateFramePlacement(model_, data_, model_.getFrameId("L_foot"));//.inverse()
        // * pinocchio::updateFramePlacement(model_, data_, model_.getFrameId("backpack"));
    // pinocchio::SE3 T_right_foot_backpack = pinocchio::updateFramePlacement(model_, data_, model_.getFrameId("R_foot"));//.inverse()
        // * pinocchio::updateFramePlacement(model_, data_, model_.getFrameId("backpack"));
    // pinocchio::SE3 T_left_foot_backpack = pinocchio::updateFramePlacement(model_, data_, model_.getFrameId("L_foot"));
    // pinocchio::SE3 T_right_foot_backpack = pinocchio::updateFramePlacement(model_, data_, model_.getFrameId("R_foot"));
    // pinocchio::SE3 T_left_foot_backpack = pinocchio::updateFramePlacement(model_, data_, model_.getFrameId("backpack")).inverse() *
    //     pinocchio::updateFramePlacement(model_, data_, model_.getFrameId("L_foot"));
    // pinocchio::SE3 T_right_foot_backpack = pinocchio::updateFramePlacement(model_, data_, model_.getFrameId("backpack")).inverse() *
    //     pinocchio::updateFramePlacement(model_, data_, model_.getFrameId("R_foot"));
    pinocchio::SE3 T_backpack_left_foot = data_.oMf[model_.getFrameId("backpack")].inverse() * data_.oMf[model_.getFrameId("L_foot")];
    pinocchio::SE3 T_backpack_right_foot = data_.oMf[model_.getFrameId("backpack")].inverse() * data_.oMf[model_.getFrameId("R_foot")];

    Eigen::Vector3d left_foot_position = T_backpack_left_foot.translation();
    Eigen::Vector3d right_foot_position = T_backpack_right_foot.translation();

    for (int i = 0; i < 3; i++)
    {
        feet_positions.push_back(left_foot_position(i));
    }
    for (int i = 0; i < 3; i++)
    {
        feet_positions.push_back(right_foot_position(i));
    }

    return feet_positions;
}

std::vector<double> ExoEstimator::getJacobian()
{
    std::vector<double> jacobian;

    // // int left_foot_id = model_.getFrameId("L_foot");
    // // int right_foot_id = model_.getFrameId("R_foot");
    // // int backpack_id = model_.getFrameId("backpack");
    // const int left_foot_id = model_.getJointId("left_ankle");
    // const int right_foot_id = model_.getJointId("right_ankle");
    // // const int left_foot_id = model_.getFrameId("L_foot");
    // // const int right_foot_id = model_.getId("R_foot");

    // pinocchio::Data::Matrix6x J_left_foot(6, model_.nv); 
    // pinocchio::Data::Matrix6x J_right_foot(6, model_.nv);
    // J_left_foot.setZero();
    // J_right_foot.setZero();

    // // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Forward kinematics...");
    // pinocchio::forwardKinematics(model_, data_, q_);

    // // const pinocchio::SE3 iMd_left_foot = data_.oMi[left_foot_id].actInv(data_.oMi[0]);
    // // const pinocchio::SE3 iMd_right_foot = data_.oMi[right_foot_id].actInv(data_.oMi[0]);
    // const pinocchio::SE3 iMd_backpack = data_.oMi[0];

    // // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Calculating Jacobians...");
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Left toe id: %d", model_.getJointId("left_toe"));
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Right toe id: %d", model_.getJointId("right_toe"));
    //     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Left foot id: %d", left_foot_id);
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Right foot id: %d", right_foot_id);
    // pinocchio::computeJointJacobian(model_, data_, q_, left_foot_id, J_left_foot);
    // pinocchio::computeJointJacobian(model_, data_, q_, right_foot_id, J_right_foot);

    // pinocchio::Data::Matrix6 Jlog_left_foot;
    // pinocchio::Data::Matrix6 Jlog_right_foot;
    // pinocchio::Jlog6(iMd_backpack.inverse(), Jlog_left_foot);
    // pinocchio::Jlog6(iMd_backpack.inverse(), Jlog_right_foot);
    // J_left_foot = -Jlog_left_foot * J_left_foot;
    // J_right_foot = -Jlog_right_foot * J_right_foot;
    // // pinocchio::computeJointJacobian(model_, data_, q_, left_foot_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_left_foot);
    // // pinocchio::computeJointJacobian(model_, data_, q_, right_foot_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_right_foot);

    // Calculating Jacobians using KDL
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Calculating Jacobians using KDL...");
    KDL::ChainJntToJacSolver kdl_solver_left_foot_ = KDL::ChainJntToJacSolver(kdl_chain_left_foot_);
    KDL::ChainJntToJacSolver kdl_solver_right_foot_ = KDL::ChainJntToJacSolver(kdl_chain_right_foot_);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "KDL solvers initialized");
    KDL::JntArray kdl_q_left_foot = KDL::JntArray(kdl_chain_left_foot_.getNrOfJoints());
    KDL::JntArray kdl_q_right_foot = KDL::JntArray(kdl_chain_right_foot_.getNrOfJoints());
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "KDL joint arrays initialized");
    for (int i = 0; i < 4; i++)
    {
        kdl_q_left_foot(i) = q_(i);
        kdl_q_right_foot(i) = q_(4+i);
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Left foot joint array: %f, %f, %f, %f", kdl_q_left_foot(0), kdl_q_left_foot(1), kdl_q_left_foot(2), kdl_q_left_foot(3));
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Right foot joint array: %f, %f, %f, %f", kdl_q_right_foot(0), kdl_q_right_foot(1), kdl_q_right_foot(2), kdl_q_right_foot(3));
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "KDL joint arrays filled");
    KDL::Jacobian kdl_jacobian_left_foot = KDL::Jacobian(kdl_chain_left_foot_.getNrOfJoints());
    KDL::Jacobian kdl_jacobian_right_foot = KDL::Jacobian(kdl_chain_right_foot_.getNrOfJoints());
    // KDL::Jacobian::setZero(kdl_jacobian_left_foot);
    // KDL::Jacobian::setZero(kdl_jacobian_right_foot);
    kdl_jacobian_left_foot.data.setZero();
    kdl_jacobian_right_foot.data.setZero();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "KDL Jacobians initialized");
    kdl_solver_left_foot_.JntToJac(kdl_q_left_foot, kdl_jacobian_left_foot);
    kdl_solver_right_foot_.JntToJac(kdl_q_right_foot, kdl_jacobian_right_foot);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "KDL Jacobians calculated");

    for (int i = 0; i < 6; i++)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Left foot Jacobian row %d: %f, %f, %f, %f", i, kdl_jacobian_left_foot.data(i, 0), kdl_jacobian_left_foot.data(i, 1), kdl_jacobian_left_foot.data(i, 2), kdl_jacobian_left_foot.data(i, 3));
    }
    for (int i = 0; i < 6; i++)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Right foot Jacobian row %d: %f, %f, %f, %f", i, kdl_jacobian_right_foot.data(i, 0), kdl_jacobian_right_foot.data(i, 1), kdl_jacobian_right_foot.data(i, 2), kdl_jacobian_right_foot.data(i, 3));
    }

    for (int j = 0; j < 4; j++)
    {
        for (int i = 0; i < 3; i++)
        {
            // jacobian.push_back(J_left_foot(i, j));
            jacobian.push_back(kdl_jacobian_left_foot.data(i, j));
        }
        for (int i = 0; i < 3; i++)
        {
            jacobian.push_back(0.0);
        }
    }
    for (int j = 0; j < 4; j++)
    {
        for (int i = 0; i < 3; i++)
        {
            jacobian.push_back(0.0);
        }
        for (int i = 0; i < 3; i++)
        {
            // jacobian.push_back(J_right_foot(i, j));
            jacobian.push_back(kdl_jacobian_right_foot.data(i, j));
        }
    }
    // jacobian = std::vector<double>(kdl_jacobian_left_foot.data.data(), kdl_jacobian_left_foot.data.data() + kdl_jacobian_left_foot.data.rows() * kdl_jacobian_left_foot.data.cols());

    Eigen::MatrixXd jacobianMatrix = Eigen::MatrixXd::Zero(6, 8);
    jacobianMatrix = Eigen::Map<Eigen::MatrixXd>(jacobian.data(), 6, 8);
    for (int i = 0; i < 6; i++)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Jacobian row %d: %f, %f, %f, %f, %f, %f, %f, %f", i, jacobianMatrix(i, 0), jacobianMatrix(i, 1), jacobianMatrix(i, 2), jacobianMatrix(i, 3), jacobianMatrix(i, 4), jacobianMatrix(i, 5), jacobianMatrix(i, 6), jacobianMatrix(i, 7));
    }

    return jacobian;
}