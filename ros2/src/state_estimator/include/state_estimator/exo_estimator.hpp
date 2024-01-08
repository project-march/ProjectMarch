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

#include "tf2_kdl/tf2_kdl.h"
#include "kdl_parser/kdl_parser.hpp"
#include "kdl/chain.hpp"
#include "kdl/chainjnttojacsolver.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"

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
        void resetJointPositions();

    private:
        void initialize_model();

        pinocchio::Model model_;
        pinocchio::Data data_;
        std::string urdf_path_;
        Eigen::VectorXd q_;

        KDL::Tree kdl_tree_;
        std::string root_link_;
        std::string left_foot_link_;
        std::string right_foot_link_;
        KDL::Chain kdl_chain_left_foot_;
        KDL::Chain kdl_chain_right_foot_;

        bool joint_positions_reset_; // TODO: Change into list of joint positions with time.

        // rclcpp::TimerBase::SharedPtr timer_;
        // void timer_callback();
};

#endif // STATE_ESTIMATOR__EXO_ESTIMATOR_HPP_