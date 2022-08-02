#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <map>
#include <march_gazebo_plugins/com_controller_node.h>
#include <string>
#include <thread>

#ifndef MARCH_GAZEBO_PLUGINS_COM_CONTROLLER_PLUGIN_H
#define MARCH_GAZEBO_PLUGINS_COM_CONTROLLER_PLUGIN_H

namespace gazebo {
class ComControllerPlugin : public ModelPlugin {
public:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) override;
    void onUpdate();
    static void startRos(const std::shared_ptr<ComControllerNode>& node)
    {
        rclcpp::spin(node);
    };

private:
    physics::ModelPtr model_;
    event::ConnectionPtr update_connection_;
    std::shared_ptr<ObstacleController> controller_;
    std::shared_ptr<ComControllerNode> node_;
    std::thread ros_thread;
    std::map<std::string, int> pd_values_;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ComControllerPlugin)
} // namespace gazebo

#endif // MARCH_GAZEBO_PLUGINS_COM_CONTROLLER_PLUGIN_H