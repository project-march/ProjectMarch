#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <march_gazebo_plugins/com_controller_node.h>
#include <march_gazebo_plugins/walk_controller.h>
#include <thread>

#ifndef MARCH_GAZEBO_PLUGINS_COM_CONTROLLER_PLUGIN_H
#define MARCH_GAZEBO_PLUGINS_COM_CONTROLLER_PLUGIN_H

namespace gazebo {
class ComControllerPlugin : public ModelPlugin {
public:
    // ComControllerPlugin();
    void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) override;
    void onUpdate();
    static void startRos(std::shared_ptr<ComControllerNode> node)
    {
        std::cout << "START SPIN..." << std::endl;
        rclcpp::spin(node);
    };

private:
    physics::ModelPtr model_;
    event::ConnectionPtr update_connection_;
    std::unique_ptr<ObstacleController> controller_;
    std::shared_ptr<ComControllerNode> node_;
    std::thread ros_thread;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ComControllerPlugin)
} // namespace gazebo

#endif // MARCH_GAZEBO_PLUGINS_COM_CONTROLLER_PLUGIN_H