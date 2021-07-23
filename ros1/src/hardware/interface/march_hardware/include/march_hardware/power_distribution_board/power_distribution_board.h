#ifndef MARCH_POWER_DISTRIBUTION_BOARD_H
#define MARCH_POWER_DISTRIBUTION_BOARD_H

#include <march_hardware/ethercat/slave.h>
#include <string>
#include "march_hardware/ethercat/pdo_types.h"

namespace march {

struct PowerDistributionBoardData {
    bit32 emergency_button_state;
    bit32 pdb_current;
    bit32 hv_total_current;
    bit32 stop_button_state;
    bit32 hv1_current;
    bit32 hv2_current;
    bit32 hv3_current;
    bit32 hv4_current;
    bit32 lv1_current;
    bit32 lv2_current;
    bit32 lv1_ok;
    bit32 lv2_ok;
    bit32 battery_percentage;
    bit32 battery_voltage;
    bit32 battery_temperature;
};

class PowerDistributionBoard : public Slave {
public:
    PowerDistributionBoard();
    PowerDistributionBoard(const Slave& slave, uint8_t byte_offset);

    PowerDistributionBoardData read();

    constexpr static unsigned int POWER_DISTRIBUTION_BOARD_DATA_LENGTH = 15;

private:
    const uint8_t byte_offset_;
};

} // namespace march

#endif // MARCH_POWER_DISTRIBUTION_BOARD_H
