/// @author Marco Bak - M8
#ifndef BUILD_MARCH_PRESSURE_SOLE_BROADCASTER_HPP
#define BUILD_MARCH_PRESSURE_SOLE_BROADCASTER_HPP

#include "controller_interface/controller_interface.hpp"

namespace march_pressure_sole_broadcaster{

class PressureSoleBroadcaster : public controller_interface::ControllerInterface {
public:
    PressureSoleBroadcaster();

    controller_interface::return_type init(const std::string& controller_name) override;

private:
    std::shared_ptr<rclcpp::Logger> logger_;

};
}
#endif //BUILD_MARCH_PRESSURE_SOLE_BROADCASTER_HPP
