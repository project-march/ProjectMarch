#include <march_hardware/pressure_sole/pressure_sole.h>

namespace march {
PressureSole::PressureSole(
    const Slave& slave, uint8_t byte_offset, std::string side)
    : Slave(slave)
    , byte_offset_(byte_offset)
    , side_(std::move(side)) {};

std::string PressureSole::getSide()
{
    return side_;
}

PressureSoleData PressureSole::read()
{
    std::array<float, PRESSURE_SOLE_DATA_LENGTH> data {};
    for (unsigned int i = 0; i < data.size(); i++) {
        // Increment the offset by 4 bytes each iteration
        data[i] = this->read32(byte_offset_ + i * sizeof(float)).f;
    }

    static_assert(PRESSURE_SOLE_DATA_LENGTH == 8);
    return { data[0], data[1], data[2], data[3], data[4], data[5], data[6],
        data[7] };
}
} // namespace march
