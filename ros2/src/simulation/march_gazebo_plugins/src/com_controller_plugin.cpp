#include <march_gazebo_plugins/com_controller_plugin.h>

namespace gazebo {
// The documentation on the CoM controller Plugin can be found at
// https://docs.projectmarch.nl/doc/march_packages/march_simulation.html#com-controller-plugin
void TestPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
{
    // Store the pointer to the model
    this->model = _parent;

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&TestPlugin::onUpdate, this));

    counter = 0;
}

// Called by the world update start event
void TestPlugin::onUpdate()
{
        // Apply a small linear velocity to the model.
        if (counter < 5000) {
            this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
            counter++;
        } else {
            counter++;
        }
        if (counter > 10000) {
            counter = 0;
        }
}

} // namespace gazebo

int main()
{
  return 0;
}