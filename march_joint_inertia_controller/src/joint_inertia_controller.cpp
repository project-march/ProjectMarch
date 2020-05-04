// Copyright 2020 Project March.
#include "my_controller.h"

namespace joint_inertia_controller_ns {
    //Controller initialization
    bool InertiaController::init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle &n)
    {
        //Retrieve the joint object to control
        std::string joint_name;
        if( !nh.getParam( "joint_name", joint_name ) ) {
            ROS_ERROR("No joint_name specified");
            return false;
        }
        joint_ = hw->getHandle("shoulder_pan_joint");
        return true;
    }

    //Controller startup
    void InertiaController::starting(const ros::Time& time) {
        //Get initial position to use in the control procedure
        init_pos_ = joint_.getPosition();
    }

    //Controller running
    void InertiaController::update(const ros::Time& time, const ros::Duration& period)
    {
        //---Perform a sinusoidal motion for joint shoulder_pan_joint
        double dpos = init_pos_ + 10 * sin(ros::Time::now().toSec());
        double cpos = joint_.getPosition();
        joint_.setCommand( -10*(cpos-dpos)); //Apply command to the selected joint
        //---
    }
    //Controller exiting
    void InertiaController::stopping(const ros::Time& time) { }
}
//Register the plugin: PLUGINLIB_EXPORT_CLASS(my_namespace::MyPlugin, base_class_namespace::PluginBaseClass)
PLUGINLIB_EXPORT_CLASS(joint_inertia_controller_ns::InertiaController, controller_interface::ControllerBase);

