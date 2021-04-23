// Copyright 2019 Project March.
#ifndef MARCH_HARDWARE_ODRIVE_PDOMAP_H
#define MARCH_HARDWARE_ODRIVE_PDOMAP_H
#include "march_hardware/ethercat/sdo_interface.h"
#include "march_hardware/motor_controller/odrive/odrive_state.h"

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <ros/ros.h>

namespace march {

struct ODriveObject {
    uint8_t offset; // Offset from the base slave address
    uint8_t length; // In Bits

    ODriveObject(uint8_t offset, uint8_t length)
        : offset(offset)
        , length(length) {};
};

enum class ODriveObjectName {
    // Read objects
    ActualPosition,
    ActualTorque,
    ActualVelocity,
    AxisError,
    MotorError,
    EncoderManagerError,
    EncoderError,
    ControllerError,

    // Write objects
    TargetTorque,
};

class ODrivePDOmap {
public:
    /* Compared to the IMCPDOMap, the ODrivePDOmap is much simpler.
     * All byte offsets can be stored directly and no additional logic is
     * required. We also don't need to configure the PDO at runtime, meaning we
     * need to implement fewer methods.
     */

    // Map ODriveObjectsNames to ODriveObjects, which contain the byte offset of
    // the object
    using ObjectMap = std::unordered_map<ODriveObjectName, ODriveObject>;

    static ObjectMap miso_objects_axis_0;
    static ObjectMap mosi_objects_axis_0;
    static ObjectMap miso_objects_axis_1;
    static ObjectMap mosi_objects_axis_1;

    // Get the byte offset for an ODriveObject of an axis
    static int8_t getMISOByteOffset(
        ODriveObjectName object_name, ODriveAxis axis);
    static int8_t getMOSIByteOffset(
        ODriveObjectName object_name, ODriveAxis axis);
};

} // namespace march

#endif // MARCH_HARDWARE_ODRIVE_PDOMAP_H
