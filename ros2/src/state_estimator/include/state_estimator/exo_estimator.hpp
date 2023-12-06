#ifndef STATE_ESTIMATOR__EXO_ESTIMATOR_HPP_
#define STATE_ESTIMATOR__EXO_ESTIMATOR_HPP_

#include <vector>
#include <string>

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "march_shared_msgs/srv/get_task_report.hpp"
#include "march_shared_msgs/srv/get_current_joint_positions.hpp"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

class ExoEstimator
{
    public:
        ExoEstimator();
        // ~ExoEstimator();
        void setJointPositions(std::vector<double> joint_positions);

    private:
        void initialize_model();

        void handleTaskReportRequest(const std::shared_ptr<march_shared_msgs::srv::GetTaskReport::Request> request,
            std::shared_ptr<march_shared_msgs::srv::GetTaskReport::Response> response);
        void handleJointPositionsRequest(const std::shared_ptr<march_shared_msgs::srv::GetCurrentJointPositions::Request> request,
            std::shared_ptr<march_shared_msgs::srv::GetCurrentJointPositions::Response> response);

        pinocchio::Model model_;
        pinocchio::Data data_;
        std::string urdf_path_;
        Eigen::VectorXd q_;

        rclcpp::Node::SharedPtr node_;
        rclcpp::Service<march_shared_msgs::srv::GetTaskReport>::SharedPtr task_report_service_;
        rclcpp::Service<march_shared_msgs::srv::GetCurrentJointPositions>::SharedPtr joint_positions_service_;
        // rclcpp::TimerBase::SharedPtr timer_;
        // void timer_callback();
};

#endif // STATE_ESTIMATOR__EXO_ESTIMATOR_HPP_