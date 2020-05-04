// Copyright 2019 Project March.

#include <march_gazebo_plugins/com_controller_plugin.h>
#include <typeinfo>

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
  this->model_ = _parent;
  this->controller_.reset(new WalkController(this->model_));

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(std::bind(&ComControllerPlugin::onUpdate, this));

  // Create our ROS node.
  this->ros_node_ = std::make_unique<ros::NodeHandle>("com_controller_plugin");

  // Create a named topic, and subscribe to it.
  ros::SubscribeOptions so = ros::SubscribeOptions::create<march_shared_resources::GaitActionGoal>(
      "/march/gait/schedule/goal", 1, boost::bind(&ComControllerPlugin::onRosMsg, this, _1), ros::VoidPtr(),
      &this->ros_queue_);
  this->ros_sub_ = this->ros_node_->subscribe(so);

  // Spin up the queue helper thread.
  this->ros_queue_thread_ = std::thread(std::bind(&ComControllerPlugin::queueThread, this));
}

void ComControllerPlugin::onRosMsg(const march_shared_resources::GaitActionGoalConstPtr& _msg)
{
  std::string new_gait_name = _msg->goal.name;
  if (new_gait_name.substr(0, 6) == "stairs")
  {
    if (typeid(*this->controller_) != typeid(StairsController))
    {
      ROS_INFO("Switch to stairs controller");
      this->controller_.reset(new StairsController(this->model_));
    }
  }
  else
  {
    if (typeid(*this->controller_) != typeid(WalkController))
    {
      ROS_INFO("Switch to walk controller");
      this->controller_.reset(new WalkController(this->model_));
    }
  }

  this->controller_->newSubgait(_msg);
}

// Called by the world update start event
void ComControllerPlugin::onUpdate()
{
  ignition::math::v4::Vector3<double> torque_left;
  ignition::math::v4::Vector3<double> torque_right;

  this->controller_->update(torque_left, torque_right);

  for (auto const& link : this->model_->GetLinks())
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
  while (this->ros_node_->ok())
  {
    this->ros_queue_.callAvailable(ros::WallDuration(timeout));
  }
}

}  // namespace gazebo
