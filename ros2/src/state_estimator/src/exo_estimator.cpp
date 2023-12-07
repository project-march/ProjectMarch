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
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "URDF path: %s", urdf_path_.c_str());
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Model name: %s", model_.name.c_str());
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Model nq: %d", model_.nq);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Model nv: %d", model_.nv);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Model njoints: %d", model_.njoints);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Model nframes: %d", model_.nframes);

    // q_ = Eigen::VectorXd::Zero(model_.nq);
    q_ = pinocchio::neutral(model_);
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
    // pinocchio::updateFramePlacements(m_model, m_model_data);

    pinocchio::SE3 T_left_foot_backpack = pinocchio::updateFramePlacement(model_, data_, model_.getFrameId("L_foot")).inverse()
        * pinocchio::updateFramePlacement(model_, data_, model_.getFrameId("backpack"));
    pinocchio::SE3 T_right_foot_backpack = pinocchio::updateFramePlacement(model_, data_, model_.getFrameId("R_foot")).inverse()
        * pinocchio::updateFramePlacement(model_, data_, model_.getFrameId("backpack"));

    Eigen::Vector3d left_foot_position = T_left_foot_backpack.translation();
    Eigen::Vector3d right_foot_position = T_right_foot_backpack.translation();

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

    int left_foot_id = model_.getFrameId("L_foot");
    int right_foot_id = model_.getFrameId("R_foot");
    int backpack_id = model_.getFrameId("backpack");

    pinocchio::Data::Matrix6x J_left_foot(6, model_.nv); 
    pinocchio::Data::Matrix6x J_right_foot(6, model_.nv);
    J_left_foot.setZero();
    J_right_foot.setZero();

    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Forward kinematics...");
    pinocchio::forwardKinematics(model_, data_, q_);
    pinocchio::updateFramePlacements(model_, data_);

    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Calculating Jacobians...");
    pinocchio::computeFrameJacobian(model_, data_, q_, left_foot_id, J_left_foot);
    pinocchio::computeFrameJacobian(model_, data_, q_, right_foot_id, J_right_foot);

    for (int j = 0; j < 4; j++)
    {
        for (int i = 0; i < 3; i++)
        {
            jacobian.push_back(J_left_foot(i, j));
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
            jacobian.push_back(J_right_foot(i, j));
        }
        for (int i = 0; i < 3; i++)
        {
            jacobian.push_back(0.0);
        }
    }

    return jacobian;
}