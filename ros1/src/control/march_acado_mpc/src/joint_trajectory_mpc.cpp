#include "joint_trajectory_mpc.hpp"
#include "model_predictive_controller.hpp"
#include <pluginlib/class_list_macros.hpp>
#include "acado_common.h"

#include <iostream>
#include <string>
#include <vector>

bool ModelPredictiveControllerInterface::init(std::vector<hardware_interface::JointHandle>& joint_handles, ros::NodeHandle& nh)
{
  joint_handles_ptr_ = &joint_handles;
  num_joints_ = joint_handles.size();

  std::vector <std::string> joint_names;
  ros::param::get("/march/joint_names", joint_names);

  // Initialize the place where the MPC command will be published
  mpc_pub_ = std::make_unique<realtime_tools::RealtimePublisher<march_shared_msgs::MpcMsg>>(nh, "/march/mpc/", 10);
  initMpcMsg();

  // Initialize the model predictive controllers
  for (unsigned int i = 0; i < num_joints_; ++i)
  {
      model_predictive_controllers_.push_back(ModelPredictiveController(getQMatrix(joint_names[i])));
      model_predictive_controllers_[i].joint_name = joint_names[i];
      model_predictive_controllers_[i].init();
  }


  return true;
}

// Retrieve the Q matrix from the parameter server for a joint.
std::vector<std::vector<float>> ModelPredictiveControllerInterface::getQMatrix(std::string joint_name)
{
  int n_rows, n_cols;
  std::string parameter_path = "/march/controller/trajectory";
  ros::param::get(parameter_path + "/q_matrices/"  + joint_name + "/n_rows", n_rows);
  ros::param::get(parameter_path + "/q_matrices/"  + joint_name + "/n_cols", n_cols);

  if (n_rows != n_cols)
  {
    ROS_WARN("Q_matrix is not square");
  }

  std::vector<float> Q_flat;
  ros::param::get(parameter_path + "/q_matrices/"  + joint_name + "/Q", Q_flat);

  // Set QMatrix for the mpc msg
  for (int i =0; i < num_joints_; ++i)
  {
    mpc_pub_->msg_.joint[i].tuning.q_matrix.assign(Q_flat.begin(), Q_flat.end());
  }

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
  if (!joint_handles_ptr_) {return;}

  // zero commands
  for (unsigned int i = 0; i < num_joints_; ++i)
  {
    (*joint_handles_ptr_)[i].setCommand(0.0);
  }
}

void ModelPredictiveControllerInterface::initMpcMsg() {
  int prediction_horizon = ACADO_N;
  mpc_pub_->msg_.joint.resize(num_joints_);

  // Loop trough all joints
  for (unsigned int i = 0; i < num_joints_; i++)
  {
    mpc_pub_->msg_.joint[i].tuning.horizon = prediction_horizon;

    mpc_pub_->msg_.joint[i].estimation.states.resize(ACADO_NX);
    mpc_pub_->msg_.joint[i].estimation.inputs.resize(ACADO_NU);

    mpc_pub_->msg_.joint[i].reference.states.resize(ACADO_NYN);
    mpc_pub_->msg_.joint[i].reference.inputs.resize(ACADO_NU);

    // Loop trough the states
    for (unsigned int j = 0; j < ACADO_NX; j++)
    {
      mpc_pub_->msg_.joint[i].estimation.states[j].array.resize(prediction_horizon + 1);
    }
    // Loop trough all the outputs
    for (unsigned int j = 0; j < ACADO_NYN; j++)
    {
//      mpc_pub_->msg_.joint[i].state.reference_trajectory[j].array.resize(prediction_horizon + 1);
      mpc_pub_->msg_.joint[i].reference.states[j].array.resize(prediction_horizon + 1);
    }

    // Loop trough all the inputs
    for (unsigned int j = 0; j < ACADO_NU; j++)
    {
      // The optimal control is one value shorter than the output,
      // since there is no control on the terminal state
      mpc_pub_->msg_.joint[i].estimation.inputs[j].array.resize(prediction_horizon);
      mpc_pub_->msg_.joint[i].reference.inputs[j].array.resize(prediction_horizon);
    }
  }
}

void ModelPredictiveControllerInterface::setMpcMsg(int joint_number)
{
  // For readability
  int i = joint_number;

  // Only set time once
  if (joint_number == 0)
  {
    mpc_pub_->msg_.header.stamp = ros::Time::now();
  }

  mpc_pub_->msg_.joint[i].diagnostics.cost = model_predictive_controllers_[i].cost;

  // Loop through the 'measurements' (y_i = 0 means 'angle', y_i = 1 means 'd/dt*angle')
  for (int n_i = 0; n_i < ACADO_N + 1; n_i++) {

    // Loop through states
    for (int x_i = 0; x_i < ACADO_NX; x_i++) {
      mpc_pub_->msg_.joint[i].estimation.states[x_i].array[n_i] =
              acadoVariables.x[ACADO_NX * n_i + x_i];
    }

    // Loop through outputs
    for (int y_i = 0; y_i < ACADO_NYN; y_i++) {
      mpc_pub_->msg_.joint[i].reference.states[y_i].array[n_i] =
              acadoVariables.y[y_i + n_i * ACADO_NY];
    }

    // Loop trough inputs
    for (int u_i = 0; u_i < ACADO_NU; u_i++) {
      mpc_pub_->msg_.joint[i].estimation.inputs[u_i].array[n_i] =
              acadoVariables.u[ACADO_NU * n_i + u_i];
      mpc_pub_->msg_.joint[i].reference.inputs[u_i].array[n_i] =
              acadoVariables.y[ACADO_NYN + u_i + n_i * ACADO_NY];
    }
  }

  // Acado solver time diagnostics
  mpc_pub_->msg_.joint[i].diagnostics.preparation_time = model_predictive_controllers_[i].t_preparation;
  mpc_pub_->msg_.joint[i].diagnostics.feedback_time = model_predictive_controllers_[i].t_feedback;
  mpc_pub_->msg_.joint[i].diagnostics.total_time = model_predictive_controllers_[i].t_preparation +
                                                   model_predictive_controllers_[i].t_feedback;

  // Acado & QPoasis error diagnostics
  mpc_pub_->msg_.joint[i].diagnostics.preparation_status = model_predictive_controllers_[i].preparationStepStatus;
  mpc_pub_->msg_.joint[i].diagnostics.feedback_status = model_predictive_controllers_[i].feedbackStepStatus;
}

void ModelPredictiveControllerInterface::updateCommand(const ros::Time& /*time*/, const ros::Duration& period,
                                                       const std::vector<joint_trajectory_controller::State>&  /*desired_states*/,
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
    state = {(*joint_handles_ptr_)[i].getPosition(), (*joint_handles_ptr_)[i].getVelocity()};
    model_predictive_controllers_[i].x0 = state;

    // Calculate mpc control signal
    model_predictive_controllers_[i].calculateControlInput();
    command = model_predictive_controllers_[i].u;

    // Apply command
    (*joint_handles_ptr_)[i].setCommand(command);
  }

  if (!mpc_pub_->trylock()) {
    return;
  }

  for (unsigned int i = 0; i < num_joints_; ++i)
  {
    // Fill MPC message with information
    setMpcMsg(i);

    // Shift the solver for next time step
    model_predictive_controllers_[i].shiftStatesAndControl();
  }

  // Publish msgs after all inputs are calculated and set
  mpc_pub_->unlockAndPublish();
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

}  // model_predictive_trajectory_controller

PLUGINLIB_EXPORT_CLASS(model_predictive_trajectory_controller::JointTrajectoryController, controller_interface::ControllerBase);
