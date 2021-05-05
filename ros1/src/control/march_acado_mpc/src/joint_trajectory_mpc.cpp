#include "joint_trajectory_mpc.hpp"
#include "acado_common.h"
#include "model_predictive_controller.hpp"
#include <pluginlib/class_list_macros.hpp>

#include <iostream>
#include <numeric>
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

    // Get the names of the joints to control
    std::vector<std::string> joint_names;
    ros::param::get("/march/joint_names", joint_names);

    // Initialize desired inputs
    desired_inputs.reserve(ACADO_NU);
    desired_inputs.resize(ACADO_NU, 0.0);

    // Initialize state and reference vectors
    initial_state.reserve(ACADO_NX);
    reference.reserve(sizeof(acadoVariables.y) / sizeof(acadoVariables.y[0]));
    end_reference.reserve(ACADO_NYN);

    // Initialize the place where the MPC message will be published
    mpc_pub_ = std::make_unique<
        realtime_tools::RealtimePublisher<march_shared_msgs::MpcMsg>>(
        nh, "/march/mpc/", 10);
    mpc_pub_->msg_.joint.resize(num_joints_);

    // Initialize the model predictive controller
    model_predictive_controller_
        = std::make_unique<ModelPredictiveController>(getWeights(joint_names));
    model_predictive_controller_->init();

    // Initialize the MPC message
    initMpcMsg();

    return true;
}

// Retrieve the weights from the parameter server for a joint.
std::vector<float> ModelPredictiveControllerInterface::getWeights(
    std::vector<std::string> joint_names)
{
    // get path to controller parameters
    std::string parameter_path = "/march/controller/trajectory";

    // Get Q and R from controller config
    std::vector<float> Q, Q_temp;
    std::vector<float> R, R_temp;
    std::vector<float> W;

    // Initialize vectors
    Q.reserve(ACADO_NX);
    R.reserve(ACADO_NU);
    W.reserve(ACADO_NY);
    JOINT_NX.reserve(num_joints_);
    JOINT_NU.reserve(num_joints_);

    for (int i = 0; i < num_joints_; i++) {

        ros::param::get(
            parameter_path + "/weights/" + joint_names[i] + "/Q", Q_temp);
        ros::param::get(
            parameter_path + "/weights/" + joint_names[i] + "/R", R_temp);

        // Add Q_temp and R_temp to Q and R respectively
        Q.insert(Q.end(), Q_temp.begin(), Q_temp.end());
        R.insert(R.end(), R_temp.begin(), R_temp.end());

        // Log Q and R sizes of each joint
        JOINT_NX.push_back(Q_temp.size());
        JOINT_NU.push_back(R_temp.size());

        // Check for validity of the weighting arrays
        ROS_ERROR_STREAM_COND(Q_temp.empty(),
            joint_names[i] << ", Q array has not been supplied or is empty");
        ROS_ERROR_STREAM_COND(R_temp.empty(),
            joint_names[i] << ", R array has not been supplied or is empty");

        // Set Q and R for the mpc msg
        mpc_pub_->msg_.joint[i].tuning.q_weights.assign(
            Q_temp.begin(), Q_temp.end());
        mpc_pub_->msg_.joint[i].tuning.r_weights.assign(
            R_temp.begin(), R_temp.end());
    }

    // Add Q and R to W
    W.insert(W.end(), Q.begin(), Q.end());
    W.insert(W.end(), R.begin(), R.end());

    ROS_ERROR_STREAM_COND(W.size() != ACADO_NY,
        "Incorrect weighting array size, size should be "
            << ACADO_NY << " but is " << W.size());

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
        (*joint_handles_ptr_)[i].setCommand(/*command=*/0.0);
    }
}

void ModelPredictiveControllerInterface::initMpcMsg()
{
    // Loop trough all joints
    for (unsigned int i = 0; i < num_joints_; i++) {
        mpc_pub_->msg_.joint[i].tuning.horizon = ACADO_N;

        mpc_pub_->msg_.joint[i].estimation.states.resize(JOINT_NX[i]);
        mpc_pub_->msg_.joint[i].estimation.inputs.resize(JOINT_NU[i]);

        mpc_pub_->msg_.joint[i].reference.states.resize(JOINT_NX[i]);
        mpc_pub_->msg_.joint[i].reference.inputs.resize(JOINT_NU[i]);

        // Loop trough the states
        for (unsigned int j = 0; j < JOINT_NX[i]; j++) {
            mpc_pub_->msg_.joint[i].estimation.states[j].array.resize(
                ACADO_N + 1);
        }
        // Loop trough all the outputs
        for (unsigned int j = 0; j < JOINT_NX[i]; j++) {
            mpc_pub_->msg_.joint[i].reference.states[j].array.resize(
                ACADO_N + 1);
        }

        // Loop trough all the inputs
        for (unsigned int j = 0; j < JOINT_NU[i]; j++) {
            // The optimal control is one value shorter than the output,
            // since there is no control on the terminal state
            mpc_pub_->msg_.joint[i].estimation.inputs[j].array.resize(ACADO_N);
            mpc_pub_->msg_.joint[i].reference.inputs[j].array.resize(ACADO_N);
        }
    }
}

void ModelPredictiveControllerInterface::setMpcMsg(int joint_number)
{
    // For readability
    int i = joint_number;

    // Get the starting column indices for the current joint (states, inputs,
    // estimation and reference) required because we can't be certain that the
    // amount of states and inputs are equal for all joints
    int col_joint_state
        = std::accumulate(JOINT_NX.begin(), JOINT_NX.begin() + i, /*init=*/0.0);
    int col_joint_input
        = std::accumulate(JOINT_NU.begin(), JOINT_NU.begin() + i, /*init=*/0.0);

    // Variables that only have to be set once
    if (joint_number == 0) {
        // Header time
        mpc_pub_->msg_.header.stamp = ros::Time::now();

        // Acado solver time diagnostics
        mpc_pub_->msg_.diagnostics.preparation_time
            = model_predictive_controller_->t_preparation;
        mpc_pub_->msg_.diagnostics.feedback_time
            = model_predictive_controller_->t_feedback;
        mpc_pub_->msg_.diagnostics.total_time
            = model_predictive_controller_->t_preparation
            + model_predictive_controller_->t_feedback;

        // Acado & QPoasis error diagnostics
        mpc_pub_->msg_.diagnostics.preparation_status
            = model_predictive_controller_->preparationStepStatus;
        mpc_pub_->msg_.diagnostics.feedback_status
            = model_predictive_controller_->feedbackStepStatus;

        // Objective function cost
        mpc_pub_->msg_.diagnostics.cost = model_predictive_controller_->cost;
    }

    // Loop through the 'measurements' (y_i = 0 means 'angle', y_i = 1 means
    // 'd/dt*angle')
    for (int n_i = 0; n_i < ACADO_N; n_i++) {

        // Set state estimation and reference
        for (int x_i = 0; x_i < JOINT_NX[i]; x_i++) {
            // Set state estimation
            mpc_pub_->msg_.joint[i].estimation.states[x_i].array[n_i]
                = acadoVariables.x[x_i + ACADO_NX * n_i + col_joint_state];

            // set state reference
            mpc_pub_->msg_.joint[i].reference.states[x_i].array[n_i]
                = acadoVariables.y[x_i + ACADO_NY * n_i + col_joint_state];
        }

        // Set input estimation and reference
        for (int u_i = 0; u_i < JOINT_NU[i]; u_i++) {
            // Set input estimation
            mpc_pub_->msg_.joint[i].estimation.inputs[u_i].array[n_i]
                = acadoVariables.u[u_i + ACADO_NU * n_i + col_joint_input];

            // Set input reference
            mpc_pub_->msg_.joint[i].reference.inputs[u_i].array[n_i]
                = acadoVariables
                      .y[u_i + ACADO_NYN + n_i * ACADO_NY + col_joint_input];
        }
    }

    // Set the terminal state estimation and reference
    for (int x_i = 0; x_i < JOINT_NX[i]; x_i++) {
        // Set state estimation
        mpc_pub_->msg_.joint[i].estimation.states[x_i].array[ACADO_N]
            = acadoVariables.x[x_i + ACADO_NX * ACADO_N + col_joint_state];

        // set state reference
        mpc_pub_->msg_.joint[i].reference.states[x_i].array[ACADO_N]
            = acadoVariables.yN[x_i + col_joint_state];
    }
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

    // Get initial state of each joint combined
    for (int i = 0; i < num_joints_; ++i) {
        initial_state.insert(initial_state.end(),
            { (*joint_handles_ptr_)[i].getPosition(),
                (*joint_handles_ptr_)[i].getVelocity() });
    }

    // Get "running" reference of each joint combined
    for (int n = 0; n < desired_states.size() - 1; ++n) {
        for (int i = 0; i < num_joints_; ++i) {
            reference.insert(reference.end(),
                { desired_states[n].position[i],
                    desired_states[n].velocity[i] });
        }
        reference.insert(
            reference.end(), desired_inputs.begin(), desired_inputs.end());
    }

    // Get "end" reference of each joint combined
    for (int i = 0; i < num_joints_; ++i) {
        end_reference.insert(end_reference.end(),
            { desired_states[ACADO_N].position[i],
                desired_states[ACADO_N].velocity[i] });
    }

    // Set initial state
    model_predictive_controller_->setInitialState(initial_state);
    initial_state.clear();

    // Set "running" reference
    // Consists of all desired states and inputs at node 0 to N-1
    model_predictive_controller_->setRunningReference(reference);
    reference.clear();

    // Set "end" reference
    // Consists of desired position and velocity at the last node N
    model_predictive_controller_->setEndReference(end_reference);
    end_reference.clear();

    // Calculate mpc and apply command
    command = model_predictive_controller_->calculateControlInput();

    for (int i = 0; i < num_joints_; ++i) {
        // Apply command
        (*joint_handles_ptr_)[i].setCommand(command[i]);

        // Fill MPC message with information
        setMpcMsg(i);
    }

    // Shift the solver for next time step
    model_predictive_controller_->shiftStatesAndControl();

    // Publish msgs after all inputs are calculated and set
    mpc_pub_->unlockAndPublish();
}

// Function that dictates what to do when the controller is stopped by the
// controller manager
void ModelPredictiveControllerInterface::stopping(const ros::Time& /*time*/)
{
    // zero commands
    for (int i = 0; i < num_joints_; ++i) {
        (*joint_handles_ptr_)[i].setCommand(/*command=*/0.0);
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
