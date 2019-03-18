#include <sstream>
#include <march_hardware_interface/march_hardware_interface.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <march_hardware/March4.h>

using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::SoftJointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;

namespace march_hardware_interface
{
    MarchHardwareInterface::MarchHardwareInterface(ros::NodeHandle& nh) : nh_(nh) {
        init();
        controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
        nh_.param("/march/hardware_interface/loop_hz", loop_hz_, 0.1);
        ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
        non_realtime_loop_ = nh_.createTimer(update_freq, &MarchHardwareInterface::update, this);
    }

    MarchHardwareInterface::~MarchHardwareInterface() {

    }

    void MarchHardwareInterface::init() {
        this->march = march4cpp::MARCH4();
        this->march.startEtherCAT();

        if (!this->march.isEthercatOperational())
        {
            ROS_FATAL("EtherCAT is not operational");
            exit(0);
        }


        // Get joint names
        nh_.getParam("/march/hardware_interface/joints", joint_names_);
        num_joints_ = joint_names_.size();

        // Resize vectors
        joint_position_.resize(num_joints_);
        joint_velocity_.resize(num_joints_);
        joint_effort_.resize(num_joints_);
        joint_position_command_.resize(num_joints_);
        joint_velocity_command_.resize(num_joints_);
        joint_effort_command_.resize(num_joints_);

        // Initialize Controller
        for (int i = 0; i < num_joints_; ++i) {
            march4cpp::Joint joint = march.getJoint(joint_names_[i]);
            // Create joint state interface
            JointStateHandle jointStateHandle(joint.getName(), &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
            joint_state_interface_.registerHandle(jointStateHandle);

            // Create position joint interface
            JointHandle jointPositionHandle(jointStateHandle, &joint_position_command_[i]);
            JointLimits limits;
            SoftJointLimits softLimits;

            getJointLimits(joint.getName(), nh_, limits);
            getSoftJointLimits(joint.getName(), nh_, softLimits);

            PositionJointSoftLimitsHandle jointLimitsHandle(jointPositionHandle, limits, softLimits);
            positionJointSoftLimitsInterface.registerHandle(jointLimitsHandle);
            position_joint_interface_.registerHandle(jointPositionHandle);

//          Set the first target as the current position.
            this->read();
            joint_position_command_[i] = joint_position_[i];

//            ROS_WARN("has position limits: %d", limits.has_position_limits);
//            ROS_WARN("min_position: %f", limits.min_position);
//            ROS_WARN("max_position: %f", limits.max_position);

            // Create effort joint interface
            JointHandle jointEffortHandle(jointStateHandle, &joint_effort_command_[i]);
            effort_joint_interface_.registerHandle(jointEffortHandle);

            joint.getIMotionCube().goToOperationEnabled();

        }

        registerInterface(&joint_state_interface_);
        registerInterface(&position_joint_interface_);
        registerInterface(&effort_joint_interface_);
        registerInterface(&positionJointSoftLimitsInterface);
    }

    void MarchHardwareInterface::update(const ros::TimerEvent& e) {
        elapsed_time_ = ros::Duration(e.current_real - e.last_real);
        read();
        controller_manager_->update(ros::Time::now(), elapsed_time_);
        write(elapsed_time_);
    }

    void MarchHardwareInterface::read() {
        for (int i = 0; i < num_joints_; i++) {
            joint_position_[i] = march.getJoint(joint_names_[i]).getAngleRad();
            ROS_INFO("Position: %f", joint_position_[i]);
        }
    }

    void MarchHardwareInterface::write(ros::Duration elapsed_time) {

        ROS_INFO_THROTTLE(1, "Trying to actuate joint %s, to %f rad, %f speed, %f effort.", joint_names_[0].c_str(), joint_position_command_[0], joint_velocity_command_[0], joint_effort_command_[0]);
        positionJointSoftLimitsInterface.enforceLimits(elapsed_time);

        for (int i = 0; i < num_joints_; i++) {
            ROS_INFO_THROTTLE(1, "Trying to actuate joint %s, to %f rad, %f speed, %f effort.", joint_names_[i].c_str(), joint_position_command_[i], joint_velocity_command_[i], joint_effort_command_[i]);
            march.getJoint(joint_names_[i]).actuateRad(joint_position_command_[i]);
        }
    }
}