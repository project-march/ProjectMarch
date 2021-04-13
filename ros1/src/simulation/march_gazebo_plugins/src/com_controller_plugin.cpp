// Copyright 2019 Project March.

#include "march_shared_msgs/ChangeComLevel.h"
#include "march_shared_msgs/GetPossibleComLevels.h"
#include <boost/bind.hpp>
#include <march_gazebo_plugins/com_controller_plugin.h>
#include <typeinfo>

namespace gazebo {
// The documentation on the CoM controller Plugin can be found at
// https://docs.projectmarch.nl/doc/march_packages/march_simulation.html#com-controller-plugin
void ComControllerPlugin::Load(
    physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
{
    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized()) {
        ROS_FATAL_STREAM(
            "A ROS node for Gazebo has not been initialized, unable to load "
            "plugin. "
            << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in "
               "the gazebo_ros package)");
        return;
    }

    // Initialise variables
    model_ = _parent;
    controller_ = std::make_unique<WalkController>(model_);

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&ComControllerPlugin::onUpdate, this));

    // Create our ROS node.
    ros_node_ = std::make_unique<ros::NodeHandle>("com_controller_plugin");

    // Create a named topic, and subscribe to it.
    ros::SubscribeOptions so
        = ros::SubscribeOptions::create<march_shared_msgs::CurrentGait>(
            "/march/gait_selection/current_gait", 1,
            boost::bind(&ComControllerPlugin::onRosMsg, this, _1),
            ros::VoidPtr(), &ros_queue_);
    ros_sub_ = ros_node_->subscribe(so);

    bool balance;
    if (ros_node_->param("/balance", balance, false)) {
        //     Create change_com_level service
        ros::AdvertiseServiceOptions aso_change_com;
        boost::function<bool(march_shared_msgs::ChangeComLevel::Request&,
            march_shared_msgs::ChangeComLevel::Response&)>
            change_com_cb
            = boost::bind(&ComControllerPlugin::onChangeComLevel, this, _1, _2);
        aso_change_com.init("/march/balance/change_com_level", change_com_cb);
        aso_change_com.callback_queue = &ros_queue_;
        change_com_level_service_
            = this->ros_node_->advertiseService(aso_change_com);

        // Create get_possible_com_levels
        ros::AdvertiseServiceOptions aso_get_com;
        boost::function<bool(march_shared_msgs::GetPossibleComLevels::Request&,
            march_shared_msgs::GetPossibleComLevels::Response&)>
            get_com_cb = boost::bind(
                &ComControllerPlugin::onGetPossibleComLevels, this, _1, _2);
        aso_get_com.init("/march/balance/get_possible_com_levels", get_com_cb);
        aso_get_com.callback_queue = &ros_queue_;
        get_possible_com_levels_service_
            = this->ros_node_->advertiseService(aso_get_com);
    }

    // Spin up the queue helper thread.
    ros_queue_thread_
        = std::thread(std::bind(&ComControllerPlugin::queueThread, this));
}

void ComControllerPlugin::onRosMsg(
    const march_shared_msgs::CurrentGaitConstPtr& msg)
{
    controller_->newSubgait(msg);
}

// Called by the world update start event
void ComControllerPlugin::onUpdate()
{
    ignition::math::v6::Vector3<double> torque_left;
    ignition::math::v6::Vector3<double> torque_right;

    controller_->update(torque_left, torque_right);

    for (auto const& link : model_->GetLinks()) {
        if (link->GetName().find("left") != std::string::npos) {
            link->AddTorque(torque_left);
        } else if (link->GetName().find("right") != std::string::npos) {
            link->AddTorque(torque_right);
        }
    }
}

void ComControllerPlugin::queueThread()
{
    static const double timeout = 0.01;
    while (ros_node_->ok()) {
        ros_queue_.callAvailable(ros::WallDuration(timeout));
    }
}

bool ComControllerPlugin::onChangeComLevel(
    march_shared_msgs::ChangeComLevel::Request& req,
    march_shared_msgs::ChangeComLevel::Response& res)
{
    ROS_INFO_STREAM("Requested to change CoM level.");
    res.success = controller_->changeComLevel(req.level_name);
    return true;
}

bool ComControllerPlugin::onGetPossibleComLevels(
    march_shared_msgs::GetPossibleComLevels::Request& req,
    march_shared_msgs::GetPossibleComLevels::Response& res)
{
    res.com_levels = controller_->com_levels;
    return true;
}

} // namespace gazebo
