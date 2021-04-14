#include "joint_trajectory_mpc.hpp"
#include "acado_common.h"
#include "model_predictive_controller.hpp"
#include <pluginlib/class_list_macros.hpp>

#include <iostream>
#include <string>
#include <vector>

// WARNING! UNSAFE!
// If you initialize this object, YOU, the caller, have to ensure
// that both `joint_handles` and `nh` have a lifetime that is at
// least as long as the lifetime of this object. Otherwise,
// undefined behavior WILL happen.
bool ModelPredictiveControllerInterface::init(
    std::vector<hardware_interface::JointHandle>& joint_handles,
    ros::NodeHandle& nh)
{
    joint_handles_ptr_ = &joint_handles;
    num_joints_ = joint_handles.size();

    std::vector<std::string> joint_names;
    ros::param::get("/march/joint_names", joint_names);

    // Initialize the place where the MPC command will be published
    mpc_pub_ = std::make_unique<
        realtime_tools::RealtimePublisher<march_shared_msgs::MpcMsg>>(
        nh, "/march/mpc/", 10);
    initMpcMsg();

    // Initialize the model predictive controllers
    for (unsigned int i = 0; i < num_joints_; ++i) {
        model_predictive_controllers_.push_back(
            ModelPredictiveController(getWeights(joint_names[i])));
        model_predictive_controllers_[i].joint_name = std::move(joint_names[i]);
        model_predictive_controllers_[i].init();
    }

    return true;
}

// Retrieve the weights from the parameter server for a joint.
std::vector<float> ModelPredictiveControllerInterface::getWeights(
    std::string joint_name)
{
    // get path to controller parameters
    std::string parameter_path = "/march/controller/trajectory";

    // Get Q and R from controller config
    std::vector<float> Q;
    std::vector<float> R;

    ros::param::get(parameter_path + "/weights/" + joint_name + "/Q", Q);
    ros::param::get(parameter_path + "/weights/" + joint_name + "/R", R);

    // Add Q and R to W
    std::vector<float> W;
    W.insert(W.begin(), Q.begin(), Q.end());
    W.insert(W.end(), R.begin(), R.end());

    // Check for validity of the weighting arrays
    ROS_WARN_STREAM_COND(
        Q.empty(), joint_name << ", Q array has not been supplied or is empty");
    ROS_WARN_STREAM_COND(
        R.empty(), joint_name << ", R array has not been supplied or is empty");
    ROS_WARN_STREAM_COND(
        W.size() != ACADO_NY, joint_name << ", Incorrect weighting array size");

    // Set WArray for the mpc msg
    for (int i = 0; i < num_joints_; ++i) {
        mpc_pub_->msg_.joint[i].tuning.weights.assign(W.begin(), W.end());
    }

    return W;
}

// Function that dictates what to do when the controller is started by the
// controller manager
void ModelPredictiveControllerInterface::starting(const ros::Time& /*time*/)
{
    if (!joint_handles_ptr_) {
        return;
    }

    // zero commands
    for (unsigned int i = 0; i < num_joints_; ++i) {
        (*joint_handles_ptr_)[i].setCommand(0.0);
    }
}

void ModelPredictiveControllerInterface::initMpcMsg()
{
    int prediction_horizon = ACADO_N;
    mpc_pub_->msg_.joint.resize(num_joints_);

    // Loop trough all joints
    for (unsigned int i = 0; i < num_joints_; i++) {
        mpc_pub_->msg_.joint[i].tuning.horizon = prediction_horizon;

        mpc_pub_->msg_.joint[i].estimation.states.resize(ACADO_NX);
        mpc_pub_->msg_.joint[i].estimation.inputs.resize(ACADO_NU);

        mpc_pub_->msg_.joint[i].reference.states.resize(ACADO_NYN);
        mpc_pub_->msg_.joint[i].reference.inputs.resize(ACADO_NU);

        // Loop trough the states
        for (unsigned int j = 0; j < ACADO_NX; j++) {
            mpc_pub_->msg_.joint[i].estimation.states[j].array.resize(
                prediction_horizon + 1);
        }
        // Loop trough all the outputs
        for (unsigned int j = 0; j < ACADO_NYN; j++) {
            mpc_pub_->msg_.joint[i].reference.states[j].array.resize(
                prediction_horizon + 1);
        }

        // Loop trough all the inputs
        for (unsigned int j = 0; j < ACADO_NU; j++) {
            // The optimal control is one value shorter than the output,
            // since there is no control on the terminal state
            mpc_pub_->msg_.joint[i].estimation.inputs[j].array.resize(
                prediction_horizon);
            mpc_pub_->msg_.joint[i].reference.inputs[j].array.resize(
                prediction_horizon);
        }
    }
}

void ModelPredictiveControllerInterface::setMpcMsg(int joint_number)
{
    // For readability
    int i = joint_number;

    // Only set time once
    if (joint_number == 0) {
        mpc_pub_->msg_.header.stamp = ros::Time::now();
    }

    mpc_pub_->msg_.joint[i].diagnostics.cost
        = model_predictive_controllers_[i].cost;

    // Loop through the 'measurements' (y_i = 0 means 'angle', y_i = 1 means
    // 'd/dt*angle')
    for (int n_i = 0; n_i < ACADO_N + 1; n_i++) {

        // Loop through states
        for (int x_i = 0; x_i < ACADO_NX; x_i++) {
            mpc_pub_->msg_.joint[i].estimation.states[x_i].array[n_i]
                = acadoVariables.x[ACADO_NX * n_i + x_i];
        }

        // Loop through outputs
        for (int y_i = 0; y_i < ACADO_NYN; y_i++) {
            mpc_pub_->msg_.joint[i].reference.states[y_i].array[n_i]
                = acadoVariables.y[y_i + n_i * ACADO_NY];
        }

        // Loop trough inputs
        for (int u_i = 0; u_i < ACADO_NU; u_i++) {
            mpc_pub_->msg_.joint[i].estimation.inputs[u_i].array[n_i]
                = acadoVariables.u[ACADO_NU * n_i + u_i];
            mpc_pub_->msg_.joint[i].reference.inputs[u_i].array[n_i]
                = acadoVariables.y[ACADO_NYN + u_i + n_i * ACADO_NY];
        }
    }

    // Acado solver time diagnostics
    mpc_pub_->msg_.joint[i].diagnostics.preparation_time
        = model_predictive_controllers_[i].t_preparation;
    mpc_pub_->msg_.joint[i].diagnostics.feedback_time
        = model_predictive_controllers_[i].t_feedback;
    mpc_pub_->msg_.joint[i].diagnostics.total_time
        = model_predictive_controllers_[i].t_preparation
        + model_predictive_controllers_[i].t_feedback;

    // Acado & QPoasis error diagnostics
    mpc_pub_->msg_.joint[i].diagnostics.preparation_status
        = model_predictive_controllers_[i].preparationStepStatus;
    mpc_pub_->msg_.joint[i].diagnostics.feedback_status
        = model_predictive_controllers_[i].feedbackStepStatus;
}

// Function that calculates the command that needs to be send to each joint
void ModelPredictiveControllerInterface::updateCommand(
    const ros::Time& /*time*/, const ros::Duration& period,
    const std::vector<joint_trajectory_controller::State>& desired_states,
    const joint_trajectory_controller::State& state_error)
{
    // Preconditions
    if (!joint_handles_ptr_) {
        return;
    }
    assert(num_joints_ == state_error.position.size());
    assert(num_joints_ == state_error.velocity.size());

    // Update effort command
    for (unsigned int i = 0; i < num_joints_; ++i) {
        // Get current joint state
        state = { (*joint_handles_ptr_)[i].getPosition(),
            (*joint_handles_ptr_)[i].getVelocity() };
        model_predictive_controllers_[i].x0 = state;

        // Set reference
        for (int j = 0; j < desired_states.size(); ++j) {
            model_predictive_controllers_[i].setReference(j,
                { desired_states[j].position[i], // angle
                    desired_states[j].velocity[i], // angular velocity
                    0.0 }); // torque
        }

        // Calculate mpc control signal
        model_predictive_controllers_[i].calculateControlInput();
        command = model_predictive_controllers_[i].u;

        // Apply command
        (*joint_handles_ptr_)[i].setCommand(command);
    }

    for (unsigned int i = 0; i < num_joints_; ++i) {
        // Fill MPC message with information
        setMpcMsg(i);

        // Shift the solver for next time step
        model_predictive_controllers_[i].shiftStatesAndControl();
    }

    // Publish msgs after all inputs are calculated and set
    mpc_pub_->unlockAndPublish();
}

// Function that dictates what to do when the controller is stopped by the
// controller manager
void ModelPredictiveControllerInterface::stopping(const ros::Time& /*time*/)
{
    // zero commands
    for (unsigned int i = 0; i < num_joints_; ++i) {
        (*joint_handles_ptr_)[i].setCommand(0.0);
    }
}

// Exporting the controller plugin
namespace model_predictive_trajectory_controller {
typedef joint_trajectory_controller::JointTrajectoryController<
    trajectory_interface::QuinticSplineSegment<double>,
    hardware_interface::EffortJointInterface>

    JointTrajectoryController;

} // namespace model_predictive_trajectory_controller

PLUGINLIB_EXPORT_CLASS(
    model_predictive_trajectory_controller::JointTrajectoryController,
    controller_interface::ControllerBase);
