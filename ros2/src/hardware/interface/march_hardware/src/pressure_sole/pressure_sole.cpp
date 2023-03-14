#include <march_hardware/pressure_sole/pressure_sole.h>
#include "march_hardware/ethercat/odrive_pdo_map.h"
#include "march_hardware/ethercat/pdo_types.h"
#include <rclcpp/rclcpp.hpp>

namespace march {
PressureSole::PressureSole(const Slave& slave, uint8_t byte_offset, std::string side)
    : Slave(slave)
    , byte_offset_(byte_offset)
    , side_(std::move(side))
{
}

std::string PressureSole::getSide()
{
    return side_;
}

void PressureSole::read(PressureSoleData& pressure_sole_data) const
{
    std::array<bit32, 2> data {};
    data[0] = this->read32(ODrivePDOmap::getMISOByteOffset(ODriveObjectName::Heel_right, ODriveAxis::None));
    data[1] = this->read32(ODrivePDOmap::getMISOByteOffset(ODriveObjectName::Heel_left, ODriveAxis::None));
    // data[2] = this->read32(ODrivePDOmap::getMISOByteOffset(ODriveObjectName::Met1, ODriveAxis::None));
    // data[3] = this->read32(ODrivePDOmap::getMISOByteOffset(ODriveObjectName::Hallux, ODriveAxis::None));
    // data[4] = this->read32(ODrivePDOmap::getMISOByteOffset(ODriveObjectName::Met3, ODriveAxis::None));
    // data[5] = this->read32(ODrivePDOmap::getMISOByteOffset(ODriveObjectName::Toes, ODriveAxis::None));
    // data[6] = this->read32(ODrivePDOmap::getMISOByteOffset(ODriveObjectName::Met5, ODriveAxis::None));
    // data[7] = this->read32(ODrivePDOmap::getMISOByteOffset(ODriveObjectName::Arch, ODriveAxis::None));

    static_assert(PRESSURE_SOLE_DATA_LENGTH == 8);
    pressure_sole_data.update_values(data);
}
} // namespace march
