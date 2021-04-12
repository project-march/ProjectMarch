// Copyright 2019 Project March.
#include "march_hardware/ethercat/pdo_map.h"
#include "march_hardware/error/hardware_exception.h"
#include "march_hardware/ethercat/sdo_interface.h"

#include <cstdint>
#include <map>
#include <utility>

namespace march {
std::unordered_map<IMCObjectName, IMCObject> PDOmap::all_objects = {
    { IMCObjectName::StatusWord, IMCObject(0x6041, 0, 16) },
    { IMCObjectName::ActualPosition, IMCObject(0x6064, 0, 32) },
    { IMCObjectName::ActualVelocity, IMCObject(0x6069, 0, 32) },
    { IMCObjectName::MotionErrorRegister, IMCObject(0x2000, 0, 16) },
    { IMCObjectName::DetailedErrorRegister, IMCObject(0x2002, 0, 16) },
    { IMCObjectName::SecondDetailedErrorRegister, IMCObject(0x2009, 0, 16) },
    { IMCObjectName::DCLinkVoltage, IMCObject(0x2055, 0, 16) },
    { IMCObjectName::DriveTemperature, IMCObject(0x2058, 0, 16) },
    { IMCObjectName::ActualTorque, IMCObject(0x6077, 0, 16) },
    { IMCObjectName::CurrentLimit, IMCObject(0x207F, 0, 16) },
    { IMCObjectName::MotorPosition, IMCObject(0x2088, 0, 32) },
    { IMCObjectName::MotorVelocity, IMCObject(0x2087, 0, 32) },
    { IMCObjectName::ControlWord, IMCObject(0x6040, 0, 16) },
    { IMCObjectName::TargetPosition, IMCObject(0x607A, 0, 32) },
    { IMCObjectName::TargetTorque, IMCObject(0x6071, 0, 16) },
    { IMCObjectName::QuickStopDeceleration, IMCObject(0x6085, 0, 32) },
    { IMCObjectName::QuickStopOption, IMCObject(0x605A, 0, 16) },
    { IMCObjectName::MotorVoltage, IMCObject(0x2108, 3, 16) }
};

void PDOmap::addObject(IMCObjectName object_name)
{
    auto it = PDOmap::all_objects.find(object_name);
    if (it == PDOmap::all_objects.end()) {
        throw error::HardwareException(
            error::ErrorType::PDO_OBJECT_NOT_DEFINED);
    }

    if (this->PDO_objects.count(object_name) != 0) {
        ROS_WARN("IMC object %i is already added to PDO map",
            static_cast<int>(object_name));
        return;
    }

    this->PDO_objects.insert(
        { object_name, it->second }); // NOLINT(whitespace/braces)
    this->total_used_bits += it->second.length;

    if (total_used_bits > this->nr_of_regs * this->bits_per_register) {
        throw error::HardwareException(error::ErrorType::PDO_REGISTER_OVERFLOW,
            "PDO object: %i could not be added (total bits %d, only %d "
            "allowed)",
            total_used_bits, (this->nr_of_regs * this->bits_per_register),
            static_cast<int>(object_name));
    }
}

std::unordered_map<IMCObjectName, uint8_t> PDOmap::map(
    SdoSlaveInterface& sdo, DataDirection direction)
{
    switch (direction) {
        case DataDirection::MISO:
            return configurePDO(sdo, 0x1A00, 0x1C13);
        case DataDirection::MOSI:
            return configurePDO(sdo, 0x1600, 0x1C12);
        default:
            ROS_ERROR("Invalid data direction given, returning empty map");
            return {};
    }
}

std::unordered_map<IMCObjectName, uint8_t> PDOmap::configurePDO(
    SdoSlaveInterface& sdo, int base_register, uint16_t base_sync_manager)
{
    int counter = 1;
    int current_register = base_register;
    int size_left = this->bits_per_register;

    std::unordered_map<IMCObjectName, uint8_t> byte_offsets;
    std::vector<std::pair<IMCObjectName, IMCObject>> sorted_PDO_objects
        = this->sortPDOObjects();

    sdo.write<uint8_t>(current_register, 0, 0);
    for (const auto& next_object : sorted_PDO_objects) {
        if (size_left - next_object.second.length < 0) {
            // PDO is filled so it can be enabled again
            sdo.write<uint8_t>(current_register, 0, counter - 1);

            // Update the sync manager with the just configured PDO
            sdo.write<uint8_t>(base_sync_manager, 0, 0);
            int current_pdo_nr = (current_register - base_register) + 1;
            sdo.write<uint16_t>(
                base_sync_manager, current_pdo_nr, current_register);

            // Move to the next PDO register by incrementing with one
            current_register++;
            if (current_register > (base_register + nr_of_regs)) {
                ROS_ERROR("Amount of registers was overwritten, amount of "
                          "parameters does not fit in the PDO messages.");
            }

            size_left = this->bits_per_register;
            counter = 1;

            sdo.write<uint8_t>(current_register, 0, 0);
        }

        int byte_offset = (current_register - base_register) * 8
            + (bits_per_register - size_left) / 8;
        byte_offsets[next_object.first] = byte_offset;

        sdo.write<uint32_t>(
            current_register, counter, next_object.second.combined_address);
        counter++;
        size_left -= next_object.second.length;
    }

    // Make sure the last PDO is activated
    sdo.write<uint8_t>(current_register, 0, counter - 1);

    // Deactivated the sync manager and configure with the new PDO
    sdo.write<uint8_t>(base_sync_manager, 0, 0);
    int current_pdo_number = (current_register - base_register) + 1;
    sdo.write<uint16_t>(
        base_sync_manager, current_pdo_number, current_register);

    // Explicitly disable PDO registers which are not used
    current_register++;
    if (current_register < (base_register + nr_of_regs)) {
        for (int unused_register = current_register;
             unused_register < (base_register + this->nr_of_regs);
             unused_register++) {
            sdo.write<uint8_t>(unused_register, 0, 0);
        }
    }

    // Activate the sync manager again
    int total_pdos = (current_register - base_register);
    sdo.write<uint8_t>(base_sync_manager, 0, total_pdos);

    return byte_offsets;
}

std::vector<std::pair<IMCObjectName, IMCObject>> PDOmap::sortPDOObjects()
{
    std::vector<std::pair<IMCObjectName, IMCObject>> sorted_PDO_objects;

    for (int objectSize : this->object_sizes) {
        for (const auto& object : PDO_objects) {
            if (object.second.length == objectSize) {
                sorted_PDO_objects.emplace_back(object);
            }
        }
    }
    return sorted_PDO_objects;
}
} // namespace march
