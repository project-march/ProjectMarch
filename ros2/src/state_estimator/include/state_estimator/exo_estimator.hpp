#ifndef STATE_ESTIMATOR__EXO_ESTIMATOR_HPP_
#define STATE_ESTIMATOR__EXO_ESTIMATOR_HPP_

#include <vector>
#include <string>

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/geometry.hpp"
#include "pinocchio/algorithm/jacobian.hpp"

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

class ExoEstimator
{
    public:
        ExoEstimator();
        // ~ExoEstimator();
        void setJointPositions(std::vector<double> joint_positions);
        std::vector<double> getFeetPositions();
        std::vector<double> getJacobian();

    private:
        void initialize_model();

        pinocchio::Model model_;
        pinocchio::Data data_;
        std::string urdf_path_;
        Eigen::VectorXd q_;

        // rclcpp::TimerBase::SharedPtr timer_;
        // void timer_callback();
};

#endif // STATE_ESTIMATOR__EXO_ESTIMATOR_HPP_