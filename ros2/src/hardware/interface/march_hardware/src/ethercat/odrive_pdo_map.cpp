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
ODrivePDOmap::ObjectMap ODrivePDOmap::miso_objects_axis_0
    = { { ODriveObjectName::AbsolutePosition, ODriveObject(/*offset=*/0, /*length=*/32) },
          { ODriveObjectName::Current, ODriveObject(/*offset=*/4, /*length=*/32) },
          { ODriveObjectName::MotorVelocity, ODriveObject(/*offset=*/8, /*length=*/32) },
          { ODriveObjectName::AxisError, ODriveObject(/*offset=*/12, /*length=*/32) },
          { ODriveObjectName::MotorError, ODriveObject(/*offset=*/16, /*length=*/32) },
          { ODriveObjectName::DieBOSlaveError, ODriveObject(/*offset=*/20, /*length=*/32) },
          { ODriveObjectName::EncoderError, ODriveObject(/*offset=*/24, /*length=*/32) },
          { ODriveObjectName::ControllerError, ODriveObject(/*offset=*/28, /*length=*/32) },
          { ODriveObjectName::AxisState, ODriveObject(/*offset=*/32, /*length=*/32) },
          { ODriveObjectName::OdriveTemperature, ODriveObject(/*offset=*/36, /*length=*/32) },
          { ODriveObjectName::MotorTemperature, ODriveObject(/*offset=*/40, /*length=*/32) },
          { ODriveObjectName::ShadowCount, ODriveObject(/*offset=*/44, /*length=*/32) },
          { ODriveObjectName::Torque, ODriveObject(/*offset=*/48, /*length=*/32) } };

ODrivePDOmap::ObjectMap ODrivePDOmap::miso_objects_axis_1
    = { { ODriveObjectName::AbsolutePosition, ODriveObject(/*offset=*/52, /*length=*/32) },
          { ODriveObjectName::Current, ODriveObject(/*offset=*/56, /*length=*/32) },
          { ODriveObjectName::MotorVelocity, ODriveObject(/*offset=*/60, /*length=*/32) },
          { ODriveObjectName::AxisError, ODriveObject(/*offset=*/64, /*length=*/32) },
          { ODriveObjectName::MotorError, ODriveObject(/*offset=*/68, /*length=*/32) },
          { ODriveObjectName::DieBOSlaveError, ODriveObject(/*offset=*/72, /*length=*/32) },
          { ODriveObjectName::EncoderError, ODriveObject(/*offset=*/76, /*length=*/32) },
          { ODriveObjectName::ControllerError, ODriveObject(/*offset=*/80, /*length=*/32) },
          { ODriveObjectName::AxisState, ODriveObject(/*offset=*/84, /*length=*/32) },
          { ODriveObjectName::OdriveTemperature, ODriveObject(/*offset=*/88, /*length=*/32) },
          { ODriveObjectName::MotorTemperature, ODriveObject(/*offset=*/92, /*length=*/32) },
          { ODriveObjectName::ShadowCount, ODriveObject(/*offset=*/96, /*length=*/32) },
          { ODriveObjectName::Torque, ODriveObject(/*offset=*/100, /*length=*/32) } };

ODrivePDOmap::ObjectMap ODrivePDOmap::miso_objects_axis_none
    = { { ODriveObjectName::OdriveError, ODriveObject(/*offset=*/104, /*length=*/32) },
          { ODriveObjectName::Heel_right, ODriveObject(/*offset=*/108, /*length=*/32) },
          { ODriveObjectName::Heel_left, ODriveObject(/*offset=*/112, /*length=*/32) },
          { ODriveObjectName::Met1, ODriveObject(/*offset=*/116, /*length=*/32) },
          { ODriveObjectName::Hallux, ODriveObject(/*offset=*/120, /*length=*/32) },
          { ODriveObjectName::Met3, ODriveObject(/*offset=*/124, /*length=*/32) },
          { ODriveObjectName::Toes, ODriveObject(/*offset=*/128, /*length=*/32) },
          { ODriveObjectName::Met5, ODriveObject(/*offset=*/132, /*length=*/32) },
          { ODriveObjectName::Arch, ODriveObject(/*offset=*/136, /*length=*/32) },
          { ODriveObjectName::Rfix, ODriveObject(/*offset=*/140, /*length=*/32) } };

ODrivePDOmap::ObjectMap ODrivePDOmap::mosi_objects_axis_0 = {
    { ODriveObjectName::TargetTorque, ODriveObject(/*offset=*/0, /*length=*/32) },
    { ODriveObjectName::TargetPosition, ODriveObject(/*offset=*/4, /*length=*/32) },
    { ODriveObjectName::FuzzyTorque, ODriveObject(/*offset=*/8, /*length=*/32) },
    { ODriveObjectName::FuzzyPosition, ODriveObject(/*offset=*/12, /*length=*/32) },
    { ODriveObjectName::PositionP, ODriveObject(/*offset=*/16, /*length=*/32) },
    { ODriveObjectName::PositionI, ODriveObject(/*offset=*/20, /*length=*/32) },
    { ODriveObjectName::PositionD, ODriveObject(/*offset=*/24, /*length=*/32) },
    { ODriveObjectName::TorqueP, ODriveObject(/*offset=*/28, /*length=*/32) },
    { ODriveObjectName::TorqueI, ODriveObject(/*offset=*/32, /*length=*/32) },
    { ODriveObjectName::TorqueD, ODriveObject(/*offset=*/36, /*length=*/32) },
    { ODriveObjectName::RequestedState, ODriveObject(/*offset=*/40, /*length=*/32) },
};

ODrivePDOmap::ObjectMap ODrivePDOmap::mosi_objects_axis_1 = {
    { ODriveObjectName::TargetTorque, ODriveObject(/*offset=*/44, /*length=*/32) },
    { ODriveObjectName::TargetPosition, ODriveObject(/*offset=*/48, /*length=*/32) },
    { ODriveObjectName::FuzzyTorque, ODriveObject(/*offset=*/52, /*length=*/32) },
    { ODriveObjectName::FuzzyPosition, ODriveObject(/*offset=*/56, /*length=*/32) },
    { ODriveObjectName::PositionP, ODriveObject(/*offset=*/60, /*length=*/32) },
    { ODriveObjectName::PositionI, ODriveObject(/*offset=*/64, /*length=*/32) },
    { ODriveObjectName::PositionD, ODriveObject(/*offset=*/68, /*length=*/32) },
    { ODriveObjectName::TorqueP, ODriveObject(/*offset=*/72, /*length=*/32) },
    { ODriveObjectName::TorqueI, ODriveObject(/*offset=*/76, /*length=*/32) },
    { ODriveObjectName::TorqueD, ODriveObject(/*offset=*/80, /*length=*/32) },
    { ODriveObjectName::RequestedState, ODriveObject(/*offset=*/84, /*length=*/32) },
};

int8_t ODrivePDOmap::getMISOByteOffset(ODriveObjectName object_name, ODriveAxis axis)
{
    switch (axis) {
        case ODriveAxis::Zero:
            return miso_objects_axis_0.at(object_name).offset;
        case ODriveAxis::One:
            return miso_objects_axis_1.at(object_name).offset;
        case ODriveAxis::None:
            return miso_objects_axis_none.at(object_name).offset;
        default:
            throw error::HardwareException(error::ErrorType::ODRIVE_WRONG_AXIS_NUMBER, "Cannot get MISO Byte offset.");
    }
}

int8_t ODrivePDOmap::getMOSIByteOffset(ODriveObjectName object_name, ODriveAxis axis)
{
    switch (axis) {
        case ODriveAxis::Zero:
            return mosi_objects_axis_0.at(object_name).offset;
        case ODriveAxis::One:
            return mosi_objects_axis_1.at(object_name).offset;
        default:
            throw error::HardwareException(error::ErrorType::ODRIVE_WRONG_AXIS_NUMBER, "Cannot get MOSI Byte offset.");
    }
}

} // namespace march
