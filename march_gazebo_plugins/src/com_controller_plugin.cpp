// Copyright 2019 Project March.

#include <com_controller_plugin.h>

namespace gazebo
{
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
  this->model = _parent;
  this->controller_.reset(new WalkController(this->model));

  this->subgait_name = "home_stand";
  this->stable_side = "left";

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection = event::Events::ConnectWorldUpdateBegin(std::bind(&ComControllerPlugin::onUpdate, this));

  // Create our ROS node.
  this->ros_node = std::make_unique<ros::NodeHandle>(ros::NodeHandle("gazebo_client"));

  // Create a named topic, and subscribe to it.
  ros::SubscribeOptions so = ros::SubscribeOptions::create<march_shared_resources::GaitActionGoal>(
      "/march/gait/schedule/goal", 1, boost::bind(&ComControllerPlugin::onRosMsg, this, _1), ros::VoidPtr(),
      &this->ros_queue);
  this->ros_sub = this->ros_node->subscribe(so);

  // Spin up the queue helper thread.
  this->ros_queue_thread = std::thread(std::bind(&ComControllerPlugin::queueThread, this));
}

void ComControllerPlugin::onRosMsg(const march_shared_resources::GaitActionGoalConstPtr& _msg)
{
  this->subgait_name = _msg->goal.current_subgait.name;
  // The left foot is stable for gaits that do not start with "left" (so also home stand etc.)
  if (this->subgait_name.substr(0, 4) == "left")
  {
    this->stable_side = "right";
  }
  else
  {
    this->stable_side = "left";
  }

  this->controller_->newSubgait(_msg);
}

// Called by the world update start event
void ComControllerPlugin::onUpdate()
{
  ignition::math::v4::Vector3<double> torque_all;     // -roll, pitch, -yawconst ignition::math::v4::Vector3<double>
                                                      // torque_all(0, T_pitch, T_yaw);
  ignition::math::v4::Vector3<double> torque_stable;  // -roll, pitch, -yaw

  this->controller_->update(torque_all, torque_stable);

  for (auto const& link : this->model->GetLinks())
  {
    link->AddTorque(torque_all);

    // Apply rolling torque to all links in the stable leg
    if (link->GetName().find(this->stable_side) != std::string::npos)
    {
      link->AddTorque(torque_stable);
    }
  }
}

void ComControllerPlugin::queueThread()
{
  static const double timeout = 0.01;
  while (this->ros_node->ok())
  {
    this->ros_queue.callAvailable(ros::WallDuration(timeout));
  }
}

}  // namespace gazebo
