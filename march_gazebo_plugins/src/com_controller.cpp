#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <march_shared_resources/GaitActionGoal.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>

namespace gazebo
{
class ComController : public ModelPlugin
{
public:
  void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) override
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
    this->foot_left = this->model->GetLink("ankle_plate_left");
    this->foot_right = this->model->GetLink("ankle_plate_right");

    this->swing_step_size = 0.7;  // This estimate will be adjusted every step
    this->subgait_name = "home_stand";

    this->p_pitch = 60;
    this->d_pitch = 10;
    this->p_roll = 150;
    this->d_roll = 7.5;
    this->p_yaw = 500;
    this->d_yaw = 25;

    this->error_x_last_timestep = 0;
    this->error_y_last_timestep = 0;
    this->error_yaw_last_timestep = 0;

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->update_connection = event::Events::ConnectWorldUpdateBegin(std::bind(&ComController::onUpdate, this));

    // Create our ROS node.
    this->ros_node = std::make_unique<ros::NodeHandle>(ros::NodeHandle("gazebo_client"));

    // Create a named topic, and subscribe to it.
    ros::SubscribeOptions so = ros::SubscribeOptions::create<march_shared_resources::GaitActionGoal>(
        "/march/gait/schedule/goal", 1, boost::bind(&ComController::onRosMsg, this, _1), ros::VoidPtr(),
        &this->ros_queue);
    this->ros_sub = this->ros_node->subscribe(so);

    // Spin up the queue helper thread.
    this->ros_queue_thread = std::thread(std::bind(&ComController::queueThread, this));
  }

  void onRosMsg(const march_shared_resources::GaitActionGoalConstPtr& _msg)
  {
    if (this->subgait_name == "right_open" or this->subgait_name == "right_swing" or this->subgait_name == "left_swing")
    {
      this->swing_step_size = 0.8 * this->swing_step_size + 0.4 * std::abs(this->foot_right->WorldPose().Pos().X() -
                                                                           this->foot_left->WorldPose().Pos().X());
    }

    this->subgait_name = _msg->goal.current_subgait.name;
    this->subgait_duration =
        _msg->goal.current_subgait.duration.sec + 0.000000001 * _msg->goal.current_subgait.duration.nsec;
    this->subgait_start_time = this->model->GetWorld()->SimTime().Double();
  }

  // Called by the world update start event
  void onUpdate()
  {
    // Note: the exo moves in the negative x direction, and the right leg is in
    // the positive y direction
    double time_since_start = this->model->GetWorld()->SimTime().Double() - subgait_start_time;
    auto model_com = this->GetCom();

    // Left foot is stable unless subgait name starts with left
    auto stable_foot_pose = this->foot_left->WorldCoGPose().Pos();
    std::string stable_side = "left";
    if (this->subgait_name.substr(0, 4) == "left")
    {
      stable_foot_pose = this->foot_right->WorldCoGPose().Pos();
      stable_side = "right";
    }

    // Goal position is determined from the location of the stable foot
    double goal_position_x = stable_foot_pose.X();
    double goal_position_y = stable_foot_pose.Y();

    // Start goal position a quarter step size behind the stable foot
    // Move the goal position forward with v = 0.5 * swing_step_size/subgait_duration
    if (this->subgait_name.substr(this->subgait_name.size() - 4) == "open")
    {
      goal_position_x += -0.25 * time_since_start * swing_step_size / subgait_duration;
    }
    else if (this->subgait_name.substr(this->subgait_name.size() - 5) == "swing")
    {
      goal_position_x += 0.25 * swing_step_size - 0.5 * time_since_start * swing_step_size / subgait_duration;
    }
    else if (this->subgait_name.substr(this->subgait_name.size() - 5) == "close")
    {
      goal_position_x += 0.25 * swing_step_size - 0.25 * time_since_start * swing_step_size / subgait_duration;
    }

    double error_x = model_com.X() - goal_position_x;
    double error_y = model_com.Y() - goal_position_y;
    double error_yaw = this->foot_left->WorldPose().Rot().Z();

    double T_pitch = -this->p_pitch * error_x - this->d_pitch * (error_x - this->error_x_last_timestep);
    double T_roll = this->p_roll * error_y + this->d_roll * (error_y - this->error_y_last_timestep);
    double T_yaw = -this->p_yaw * error_yaw - this->d_yaw * (error_yaw - this->error_yaw_last_timestep);

    const ignition::math::v4::Vector3<double> torque_all(0, T_pitch, T_yaw);  // -roll, pitch, -yaw
    const ignition::math::v4::Vector3<double> torque_stable(T_roll, 0, 0);    // -roll, pitch, -yaw

    for (auto const& link : this->model->GetLinks())
    {
      link->AddTorque(torque_all);

      // Apply rolling torque to all links in the stable leg
      if (link->GetName().find(stable_side) != std::string::npos)
      {
        link->AddTorque(torque_stable);
      }
    }

    this->error_x_last_timestep = error_x;
    this->error_y_last_timestep = error_y;
    this->error_yaw_last_timestep = error_yaw;
  }

private:
  void queueThread()
  {
    static const double timeout = 0.01;
    while (this->ros_node->ok())
    {
      this->ros_queue.callAvailable(ros::WallDuration(timeout));
    }
  }

  // Called by the world update start event
  double getMass()
  {
    double mass = 0.0;
    for (auto const& link : this->model->GetLinks())
    {
      mass += link->GetInertial()->Mass();
    }
    return mass;
  }

  // Called by the world update start event
  ignition::math::v4::Vector3<double> GetCom()
  {
    ignition::math::v4::Vector3<double> com(0.0, 0.0, 0.0);
    for (auto const& link : this->model->GetLinks())
    {
      com += link->WorldCoGPose().Pos() * link->GetInertial()->Mass();
    }
    return com / this->getMass();
  }

  physics::ModelPtr model;
  physics::LinkPtr foot_left;
  physics::LinkPtr foot_right;

  double subgait_start_time;
  double swing_step_size;
  double subgait_duration;
  std::string subgait_name;

  double p_yaw;
  double d_yaw;
  double p_pitch;
  double d_pitch;
  double p_roll;
  double d_roll;
  double error_x_last_timestep;
  double error_y_last_timestep;
  double error_yaw_last_timestep;

  // Pointer to the update event connection
  event::ConnectionPtr update_connection;

  /// \brief A node use for ROS transport
  std::unique_ptr<ros::NodeHandle> ros_node;

  /// \brief A ROS subscriber
  ros::Subscriber ros_sub;

  /// \brief A ROS callbackqueue that helps process messages
  ros::CallbackQueue ros_queue;

  /// \brief A thread the keeps running the ros_queue
  std::thread ros_queue_thread;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ComController)
}  // namespace gazebo
