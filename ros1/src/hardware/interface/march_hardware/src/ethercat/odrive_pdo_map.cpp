// Copyright 2019 Project March.
#include "march_hardware/ethercat/odrive_pdo_map.h"
#include "march_hardware/error/hardware_exception.h"
#include "march_hardware/ethercat/sdo_interface.h"

#include <cstdint>
#include <map>
#include <utility>

namespace march {
/* The offset and length values were retrieved from
 * https://confluence.projectmarch.nl:8443/display/62tech/GES+Implementation
 */
ODrivePDOmap::ObjectMap ODrivePDOmap::miso_objects_axis_0 = {
    { ODriveObjectName::ActualPosition, ODriveObject(0, 32) },
    { ODriveObjectName::ActualTorque, ODriveObject(4, 32) },
    { ODriveObjectName::ActualVelocity, ODriveObject(8, 32) },
    { ODriveObjectName::AxisError, ODriveObject(12, 32) },
    { ODriveObjectName::MotorError, ODriveObject(16, 16) },
    { ODriveObjectName::EncoderManagerError, ODriveObject(20, 32) },
    { ODriveObjectName::EncoderError, ODriveObject(24, 16) },
    { ODriveObjectName::ControllerError, ODriveObject(28, 8) },
};

ODrivePDOmap::ObjectMap ODrivePDOmap::miso_objects_axis_1
    = { { ODriveObjectName::ActualPosition, ODriveObject(32, 32) },
          { ODriveObjectName::ActualTorque, ODriveObject(36, 32) },
          { ODriveObjectName::ActualVelocity, ODriveObject(40, 32) },
          { ODriveObjectName::AxisError, ODriveObject(44, 32) },
          { ODriveObjectName::MotorError, ODriveObject(48, 16) },
          { ODriveObjectName::EncoderManagerError, ODriveObject(52, 32) },
          { ODriveObjectName::EncoderError, ODriveObject(56, 16) },
          { ODriveObjectName::ControllerError, ODriveObject(60, 8) } };

ODrivePDOmap::ObjectMap ODrivePDOmap::mosi_objects_axis_0 = {
    { ODriveObjectName::TargetTorque, ODriveObject(0, 32) },
};

ODrivePDOmap::ObjectMap ODrivePDOmap::mosi_objects_axis_1 = {
    { ODriveObjectName::TargetTorque, ODriveObject(0, 32) },
};

int8_t ODrivePDOmap::getMISOByteOffset(
    ODriveObjectName object_name, ODriveAxis axis)
{
    switch (axis) {
        case ODriveAxis::Zero:
            return miso_objects_axis_0.at(object_name).offset;
        case ODriveAxis::One:
            return miso_objects_axis_1.at(object_name).offset;
        default:
            throw error::HardwareException(
                error::ErrorType::ODRIVE_WRONG_AXIS_NUMBER,
                "Cannot get MISO Byte offset.");
    }
}

int8_t ODrivePDOmap::getMOSIByteOffset(
    ODriveObjectName object_name, ODriveAxis axis)
{
    switch (axis) {
        case ODriveAxis::Zero:
            return mosi_objects_axis_0.at(object_name).offset;
        case ODriveAxis::One:
            return mosi_objects_axis_1.at(object_name).offset;
        default:
            throw error::HardwareException(
                error::ErrorType::ODRIVE_WRONG_AXIS_NUMBER,
                "Cannot get MOSI Byte offset.");
    }
}

} // namespace march
