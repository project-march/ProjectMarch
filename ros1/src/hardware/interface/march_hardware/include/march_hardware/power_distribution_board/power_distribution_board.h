#ifndef MARCH_POWER_DISTRIBUTION_BOARD_H
#define MARCH_POWER_DISTRIBUTION_BOARD_H

#include "march_hardware/ethercat/pdo_types.h"
#include <march_hardware/ethercat/slave.h>
#include <string>

namespace march {

struct PowerDistributionBoardData {
    // Either pressed (0) or not (1)
    bit32 emergency_button_state;

    // Amount of current used by the PDB itself
    bit32 pdb_current;

    // Total high voltage net current
    bit32 hv_total_current;

    // Either pressed (0) or not (1)
    bit32 stop_button_state;

    // High voltage net currents
    bit32 hv1_current;
    bit32 hv2_current;
    bit32 hv3_current;
    bit32 hv4_current;

    // Low voltage net currents
    bit32 lv1_current;
    bit32 lv2_current;

    // Low voltage net is either on (1) or not (0)
    bit32 lv1_ok;
    bit32 lv2_ok;

    // Information about the battery
    bit32 battery_percentage;
    bit32 battery_voltage;
    bit32 battery_temperature;
};

class PowerDistributionBoard : public Slave {
public:
    PowerDistributionBoard();
    PowerDistributionBoard(const Slave& slave, uint8_t byte_offset);

    /**
     * Read all data from the etherCAT train
     * @return Returns a struct containing all data
     */
    PowerDistributionBoardData read();

    // Number of data values sent over
    constexpr static unsigned int POWER_DISTRIBUTION_BOARD_DATA_LENGTH = 15;

private:
    const uint8_t byte_offset_;
};

} // namespace march

#endif // MARCH_POWER_DISTRIBUTION_BOARD_H
