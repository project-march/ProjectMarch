
#ifndef MARCH_HARDWARE_TRAJECTORY_MPC_H
#define MARCH_HARDWARE_TRAJECTORY_MPC_H

#include <cassert>

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <joint_trajectory_controller/joint_trajectory_controller.h>
#include <pluginlib/class_list_macros.h>

#include <memory>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include "std_msgs/Float64.h"
#include <trajectory_interface/quintic_spline_segment.h>

#include <iostream>
#include <vector>
#include "acado_mpc.h"

template <typename T>
using RtPublisherPtr = std::unique_ptr<realtime_tools::RealtimePublisher<T>>;

namespace joint_trajectory_controller
{
    typedef trajectory_interface::QuinticSplineSegment<double> SegmentImpl;
    typedef JointTrajectorySegment<SegmentImpl> Segment;
    typedef typename Segment::State State;
}  // namespace joint_trajectory_controller

template <>
class HardwareInterfaceAdapter<hardware_interface::EffortJointInterface, joint_trajectory_controller::State>
{
public:
    HardwareInterfaceAdapter() : joint_handles_ptr_(nullptr) { }

    /**
     * \brief Initialize the controller by establishing the pointer to the joints
     */
    bool init(std::vector<hardware_interface::JointHandle>& joint_handles, ros::NodeHandle& nh)
    {
        joint_handles_ptr_ = &joint_handles;
        num_joints_ = joint_handles_ptr_->size();
        pub_.resize(num_joints_);

        std::cout << num_joints_ << std::endl;
        int fuck = 3;
        // do something I guess
        model_predictive_controllers_.resize(num_joints_);
        for (unsigned int i = 0; i < num_joints_; i++)
        {
            model_predictive_controllers_[i].init();
        }

        return true;
    }

    /**
    * \brief Starts the controller by checking if the joint handle pointer is filled and sets
    * the initial command to zero so that the joint doesn't start moving without a desired trajectory
    */
    void starting(const ros::Time& /*time*/)
    {
        if (!joint_handles_ptr_) {return;}

        // zero commands
        for (unsigned int i = 0; i < num_joints_; ++i)
        {
            (*joint_handles_ptr_)[i].setCommand(0.0);
        }
    }

    /**
     * \brief Updates the commanded effort for each individual joint and estimates the inertia on each joint and publishes
     * that information on a topic
     */
    void updateCommand(const ros::Time& /*time*/, const ros::Duration& period,
                       const joint_trajectory_controller::State& /*desired state*/,
                       const joint_trajectory_controller::State& state_error)
    {
        num_joints_ = joint_handles_ptr_->size();

        // Preconditions
        if (!joint_handles_ptr_)
        {
            return;
        }
        assert(num_joints_ == state_error.position.size());
        assert(num_joints_ == state_error.velocity.size());

        // Update effort command
        for (unsigned int i = 0; i < num_joints_; ++i)
        {
            const double command = state_error.position[i]*1000;
            (*joint_handles_ptr_)[i].setCommand(10);
        }

    }

    /**
     * \brief Procedure, if required, for stopping the controller
     */
    void stopping(const ros::Time& /*time*/) { }

private:
    std::vector<hardware_interface::JointHandle>* joint_handles_ptr_;

    unsigned int num_joints_;

    std::vector<RtPublisherPtr<std_msgs::Float64>> pub_;
    std_msgs::Float64 msg_;

    std::vector<ModelPredictiveController> model_predictive_controllers_;
};

#endif  // MARCH_HARDWARE_TRAJECTORY_MPC_H
