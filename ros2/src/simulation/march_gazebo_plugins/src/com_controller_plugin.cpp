#include <march_gazebo_plugins/com_controller_plugin.h>

namespace gazebo {
// Called at simulation startup:
void ComControllerPlugin::Load(
    physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
{
    // Store the pointer to the model:
    model_ = _parent;

    // Bind function that should be called every simulation iteration:
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&ComControllerPlugin::onUpdate, this));

    // Default PD values:
    pd_values_["p_roll"] = 250;
    pd_values_["d_roll"] = 7000;
    pd_values_["p_pitch"] = 220;
    pd_values_["d_pitch"] = 7000;
    pd_values_["p_yaw"] = 1500;
    pd_values_["d_yaw"] = 7000;

    // Pointer to the controller:
    controller_ = std::make_shared<ObstacleController>(model_, pd_values_);

    // Pointer to the ros node:
    node_ = std::make_shared<ComControllerNode>(controller_, pd_values_);

    // Start ros on a new thread:
    ros_thread = std::thread(&ComControllerPlugin::startRos, node_);
}

// Update event to be called every simulation iteration:
void ComControllerPlugin::onUpdate()
{
    if (! node_->balance_mode()){
    ignition::math::v6::Vector3<double> torque_left;
    ignition::math::v6::Vector3<double> torque_right;

    controller_->update(torque_left, torque_right);

    for (auto const& link : model_->GetLinks()) {
        if (link->GetName().find(/*__s=*/"left") != std::string::npos) {
            link->AddTorque(torque_left);
        } else if (link->GetName().find(/*__s=*/"right") != std::string::npos) {
            link->AddTorque(torque_right);
        }
    }
}}
} // namespace gazebo