#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <march_gazebo_plugins/walk_controller.h>

#ifndef MARCH_GAZEBO_PLUGINS_COM_CONTROLLER_PLUGIN_H
#define MARCH_GAZEBO_PLUGINS_COM_CONTROLLER_PLUGIN_H


namespace gazebo {
class ComControllerPlugin : public ModelPlugin {
public:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) override;
    void onUpdate();

private:
    physics::ModelPtr model_;
    event::ConnectionPtr update_connection_;
    std::unique_ptr<ObstacleController> controller_;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ComControllerPlugin)
} // namespace gazebo

#endif // MARCH_GAZEBO_PLUGINS_COM_CONTROLLER_PLUGIN_H