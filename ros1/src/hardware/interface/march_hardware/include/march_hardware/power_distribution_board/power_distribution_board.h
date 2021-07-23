#ifndef MARCH_POWER_DISTRIBUTION_BOARD_H
#define MARCH_POWER_DISTRIBUTION_BOARD_H

#include <march_hardware/ethercat/slave.h>
#include <string>

namespace march {
struct PowerDistributionBoardData {
    uint32_t emergency_button_state;
    float pdb_current;
    float hv_total_current;
    uint32_t stop_button_state;
    float hv1_current;
    float hv2_current;
    float hv3_current;
    float hv4_current;
    float lv1_current;
    float lv2_current;
    uint32_t lv1_ok;
    uint32_t lv2_ok;
    float battery_percentage;
    float battery_voltage;
    float battery_temperature;
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
