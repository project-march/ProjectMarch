// Copyright 2019 Project March.

#include <march_gazebo_plugins/com_controller_plugin.h>
#include <typeinfo>
#include "std_srvs/Trigger.h"


namespace gazebo
{
// The documentation on the CoM controller Plugin can be found at
// https://docs.projectmarch.nl/doc/march_packages/march_simulation.html#com-controller-plugin
void ComControllerPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
{
  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load "
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
  update_connection_ = event::Events::ConnectWorldUpdateBegin(std::bind(&ComControllerPlugin::onUpdate, this));

  // Create our ROS node.
  ros_node_ = std::make_unique<ros::NodeHandle>("com_controller_plugin");

  // Create a named topic, and subscribe to it.
  ros::SubscribeOptions so = ros::SubscribeOptions::create<march_shared_msgs::CurrentGait>(
      "/march/gait_selection/current_gait", 1, boost::bind(&ComControllerPlugin::onRosMsg, this, _1), ros::VoidPtr(),
      &ros_queue_);
  ros_sub_ = ros_node_->subscribe(so);

  // Create a service for changing balance com strength
//  bool balance;
//  if (ros_node_->getParam("/march/balance", balance)) {
//    ROS_WARN_STREAM("Balance param " << "was found.");
//  }
//  if (balance) {
//  ROS_WARN_STREAM("Balance param " << "was true.");
//  ros::ServiceServer ros_service_ = ros_node_->advertiseService("/march/balance/change_com_level", &ComControllerPlugin::onChangeComLevel, this);
//  }
  // Spin up the queue helper thread.
  ros_queue_thread_ = std::thread(std::bind(&ComControllerPlugin::queueThread, this));
}

//bool ComControllerPlugin::onChangeComLevel(std_srvs::Trigger::Request &req,
//             std_srvs::Trigger::Response &res) {
//    ROS_WARN_STREAM("Requested to change ComLevel. " << "Go go go.");
//    return false;
//}

void ComControllerPlugin::onRosMsg(const march_shared_msgs::CurrentGaitConstPtr& msg)
{
  controller_->newSubgait(msg);
}

// Called by the world update start event
void ComControllerPlugin::onUpdate()
{
  ignition::math::v6::Vector3<double> torque_left;
  ignition::math::v6::Vector3<double> torque_right;

  controller_->update(torque_left, torque_right);

  for (auto const& link : model_->GetLinks())
  {
    if (link->GetName().find("left") != std::string::npos)
    {
      link->AddTorque(torque_left);
    }
    else if (link->GetName().find("right") != std::string::npos)
    {
      link->AddTorque(torque_right);
    }
  }
}

void ComControllerPlugin::queueThread()
{
  static const double timeout = 0.01;
  while (ros_node_->ok())
  {
    ros_queue_.callAvailable(ros::WallDuration(timeout));
  }
}

}  // namespace gazebo
