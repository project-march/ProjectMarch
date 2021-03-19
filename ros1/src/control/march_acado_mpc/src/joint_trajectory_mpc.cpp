#include "joint_trajectory_mpc.hpp"
#include "model_predictive_controller.hpp"
#include <pluginlib/class_list_macros.hpp>

#include <iostream>
#include <string>
#include <vector>

bool ModelPredictiveControllerInterface::init(std::vector<hardware_interface::JointHandle>& joint_handles,
                                              ros::NodeHandle& nh)
{
  joint_handles_ptr_ = &joint_handles;
  num_joints_ = joint_handles.size();

  std::vector<std::string> joint_names;
  ros::param::get("/march/joint_names", joint_names);

  // Initialize the model predictive controllers
  for (unsigned int i = 0; i < num_joints_; ++i)
  {
    model_predictive_controllers_.push_back(ModelPredictiveController(getQMatrix(joint_names[i])));
    model_predictive_controllers_[i].joint_name = joint_names[i];
    model_predictive_controllers_[i].init();
  }

  // Initialize the place where the MPC command will be published

  command_pub_ =
      std::make_unique<realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>>(nh, "/march/mpc/command", 10);
  command_pub_->msg_.data.resize(num_joints_);
  return true;
}

// Retrieve the Q matrix from the parameter server for a joint.
std::vector<std::vector<float>> ModelPredictiveControllerInterface::getQMatrix(std::string joint_name)
{
  int n_rows, n_cols;
  std::string parameter_path = "/march/controller/trajectory";
  ros::param::get(parameter_path + "/q_matrices/" + joint_name + "/n_rows", n_rows);
  ros::param::get(parameter_path + "/q_matrices/" + joint_name + "/n_cols", n_cols);

  if (n_rows != n_cols)
  {
    ROS_WARN("Q_matrix is not square");
  }

  std::vector<float> Q_flat;
  ros::param::get(parameter_path + "/q_matrices/" + joint_name + "/Q", Q_flat);

  std::vector<std::vector<float>> Q(n_rows, std::vector<float>(n_cols));
  if (Q_flat.size() != n_rows * n_cols)
  {
    ROS_WARN("Q_matrix does not have specified matrix dimensions.");
  }
  else
  {
    for (int y = 0; y < n_rows; y++)
    {
      for (int x = 0; x < n_cols; x++)
      {
        Q[y][x] = Q_flat[y * n_cols + x];
      }
    }
  }
  return Q;
}

void ModelPredictiveControllerInterface::starting(const ros::Time& /*time*/)
{
  if (!joint_handles_ptr_)
  {
    return;
  }

  // zero commands
  for (unsigned int i = 0; i < num_joints_; ++i)
  {
    (*joint_handles_ptr_)[i].setCommand(0.0);
  }
}

void ModelPredictiveControllerInterface::updateCommand(
    const ros::Time& /*time*/, const ros::Duration& period,
    const std::vector<joint_trajectory_controller::State>& desired_states,
    const joint_trajectory_controller::State& state_error)
{
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
    // Get current joint state
    state = { (*joint_handles_ptr_)[i].getPosition(), (*joint_handles_ptr_)[i].getVelocity() };
    model_predictive_controllers_[i].x0 = state;

    // Set reference
    for (int j = 0; j < desired_states.size(); ++j)
    {
      model_predictive_controllers_[i].setReference(j, { desired_states[j].position[i],  // angle
                                                          desired_states[j].velocity[i],  // angular velocity
                                                          0.0 });                         // torque
    }

    // Calculate mpc control signal
    model_predictive_controllers_[i].calculateControlInput();
    command = model_predictive_controllers_[i].u;

    // Apply command
    (*joint_handles_ptr_)[i].setCommand(command);

    // Publish command
    if (!command_pub_->trylock())
    {
      return;
    }
    command_pub_->msg_.data[i] = command;
  }

  command_pub_->unlockAndPublish();
}

void ModelPredictiveControllerInterface::stopping(const ros::Time& /*time*/)
{
}

// Exporting the controller plugin
namespace model_predictive_trajectory_controller
{
typedef joint_trajectory_controller::JointTrajectoryController<trajectory_interface::QuinticSplineSegment<double>,
                                                               hardware_interface::EffortJointInterface>

    JointTrajectoryController;

}  // namespace model_predictive_trajectory_controller

PLUGINLIB_EXPORT_CLASS(model_predictive_trajectory_controller::JointTrajectoryController,
                       controller_interface::ControllerBase);
