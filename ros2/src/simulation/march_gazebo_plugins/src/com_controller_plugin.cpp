#include <march_gazebo_plugins/com_controller_plugin.h>

namespace gazebo {
// The documentation on the CoM controller Plugin can be found at
// https://docs.projectmarch.nl/doc/march_packages/march_simulation.html#com-controller-plugin
void ComControllerPlugin::Load(
    physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
{
    // Store the pointer to the model
    model_ = _parent;

    // Pointer to the controller
    controller_ = std::make_unique<WalkController>(model_);

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&ComControllerPlugin::onUpdate, this));
}

// Called by the world update start event
void ComControllerPlugin::onUpdate()
{
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
}
} // namespace gazebo