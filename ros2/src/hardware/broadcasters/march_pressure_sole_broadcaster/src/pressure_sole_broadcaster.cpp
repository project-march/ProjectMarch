/// @author Marco Bak - M8

#include "pressure_sole_broadcaster/pressure_sole_broadcaster.hpp"

namespace march_pressure_sole_broadcaster {

PressureSoleBroadcaster::PressureSoleBroadcaster()
        : logger_(std::make_shared<rclcpp::Logger>(rclcpp::get_logger("pressure_sole_broadcaster"))) {
}

controller_interface::return_type PressureSoleBroadcaster::init(const std::string &controller_name) {
    auto ret = ControllerInterface::init(controller_name);
    if (ret != controller_interface::return_type::OK) {
        RCLCPP_FATAL((*logger_), "Pressure sole broadcaster init was not 'OK' but was 'ERROR'.");
        return ret;
    }
    return controller_interface::return_type::OK;
}
}