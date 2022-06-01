#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

#ifndef MARCH_GAZEBO_PLUGINS_COM_CONTROLLER_PLUGIN_H
#define MARCH_GAZEBO_PLUGINS_COM_CONTROLLER_PLUGIN_H


namespace gazebo {
class TestPlugin : public ModelPlugin {
public:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) override;
    
    // Called by the world update start event
    void onUpdate();

    int counter;

private:
    physics::ModelPtr model;

    // Pointer to the update event connection
    event::ConnectionPtr update_connection_;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(TestPlugin)
} // namespace gazebo

#endif // MARCH_GAZEBO_PLUGINS_COM_CONTROLLER_PLUGIN_H