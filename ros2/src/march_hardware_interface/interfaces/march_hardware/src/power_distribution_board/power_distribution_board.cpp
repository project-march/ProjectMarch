#include "march_hardware/power_distribution_board/power_distribution_board.h"
#include "march_hardware/ethercat/pdo_types.h"

namespace march {
PowerDistributionBoard::PowerDistributionBoard(const Slave& slave, uint8_t byte_offset)
    : Slave(slave)
    , byte_offset_(byte_offset)
{
}

void PowerDistributionBoard::read(PowerDistributionBoardData& pdb_data) const
{
    std::array<bit32, pdb::DATA_LENGTH> data {};
    for (unsigned int i = 0; i < data.size(); i++) {
        // Increment the offset by 4 bytes each iteration
        data[i] = this->read32(byte_offset_ + i * sizeof(float));
    }
    pdb_data.update_values(data);
}

} // namespace march
