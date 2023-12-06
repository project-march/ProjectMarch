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

    q_ = Eigen::VectorXd::Zero(model_.nq);
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
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Joint positions set");
    for (int i = 0; i < q_.size(); i++)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Joint %d: %f", i, q_(i));
    }
}