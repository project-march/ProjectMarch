#include <march_hardware/pressure_sole/pressure_sole.h>

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
    std::array<bit32, PRESSURE_SOLE_DATA_LENGTH> data {};
    for (unsigned int i = 0; i < data.size(); i++) {
        // Increment the offset by 4 bytes each iteration
        data[i] = this->read32(byte_offset_ + i * sizeof(float));
    }

    static_assert(PRESSURE_SOLE_DATA_LENGTH == 8);
    pressure_sole_data.update_values(data);
}
} // namespace march
