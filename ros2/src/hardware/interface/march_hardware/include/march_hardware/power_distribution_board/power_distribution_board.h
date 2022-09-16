#ifndef MARCH_POWER_DISTRIBUTION_BOARD_H
#define MARCH_POWER_DISTRIBUTION_BOARD_H

#include "march_hardware/ethercat/pdo_types.h"
#include <march_hardware/ethercat/slave.h>
#include <string>

namespace march {

namespace pdb {
    inline constexpr static unsigned int DATA_LENGTH = 15;
}

struct PowerDistributionBoardData {
    // Number of data values sent over

    // Either pressed (0) or not (1)
    double emergency_button_state;

    // Amount of current used by the PDB itself
    double pdb_current;

    // Total high voltage net current
    double hv_total_current;

    // Either pressed (0) or not (1)
    double stop_button_state;

    // High voltage net currents
    double hv1_current;
    double hv2_current;
    double hv3_current;
    double hv4_current;

    // Low voltage net currents
    double lv1_current;
    double lv2_current;

    // Low voltage net is either on (1) or not (0)
    double lv1_ok;
    double lv2_ok;

    // Information about the battery
    double battery_percentage;
    double battery_voltage;
    double battery_temperature;

    /**
     * @brief Gives a pointer to the member variables so that the hardware interface can couple it back to the
     * broadcaster.
     * @attention Should contain exactly the same interface_names as expected by the
     * `march_pdb_state_broadcaster/.../pdb_semantic_component.hpp`
     * @returns A pair of interface_names and pointers to the member variables.
     */
    inline std::array<std::pair<std::string, double*>, pdb::DATA_LENGTH> get_pointers()
    {
        return { std::make_pair(/*__x=*/"emergency_button_pressed", &emergency_button_state),
            std::make_pair(/*__x=*/"pdb_current", &pdb_current),
            std::make_pair(/*__x=*/"hv_state.total_current", &hv_total_current),
            std::make_pair(/*__x=*/"hv_state.hv1_current", &hv1_current),
            std::make_pair(/*__x=*/"hv_state.hv2_current", &hv2_current),
            std::make_pair(/*__x=*/"hv_state.hv3_current", &hv3_current),
            std::make_pair(/*__x=*/"hv_state.hv4_current", &hv4_current),
            std::make_pair(/*__x=*/"stop_button_pressed", &stop_button_state),
            std::make_pair(/*__x=*/"lv_state.lv1_current", &lv1_current),
            std::make_pair(/*__x=*/"lv_state.lv2_current", &lv2_current),
            std::make_pair(/*__x=*/"lv_state.lv1_ok", &lv1_ok), std::make_pair(/*__x=*/"lv_state.lv2_ok", &lv2_ok),
            std::make_pair(/*__x=*/"battery_state.percentage", &battery_percentage),
            std::make_pair(/*__x=*/"battery_state.voltage", &battery_voltage),
            std::make_pair(/*__x=*/"battery_state.temperature", &battery_temperature) };
    }

    /**
     * @brief Updates the member variables based on the input array.
     * @attention This 0 to 14 should be in the same order as it is send in the ethercat.
     * This method is used with the output from the read method of the `PowerDistributionBoard`.
     * Make sure that the order, types (.ui or .f) and the offset for the read corresponds with position order seen via
     * the `ethercat-sdk`.
     * @param data The input array in order.
     */
    inline void update_values(std::array<bit32, pdb::DATA_LENGTH> data)
    {
        emergency_button_state = data[0].ui;
        pdb_current = data[1].f;
        hv_total_current = data[2].f;
        stop_button_state = data[3].ui;
        hv1_current = data[4].f;
        hv2_current = data[5].f;
        hv3_current = data[6].f;
        hv4_current = data[7].f;
        lv1_current = data[8].f;
        lv2_current = data[9].f;
        lv1_ok = data[10].ui;
        lv2_ok = data[11].ui;
        battery_percentage = data[12].f;
        battery_voltage = data[13].f;
        battery_temperature = data[14].f;
    }
};

class PowerDistributionBoard : public Slave {
public:
    PowerDistributionBoard();
    PowerDistributionBoard(const Slave& slave, uint8_t byte_offset);

    /// Read all data from the etherCAT train and updates the pdb_data object.
    void read(PowerDistributionBoardData& pdb_data) const;

private:
    const uint8_t byte_offset_;
};

} // namespace march

#endif // MARCH_POWER_DISTRIBUTION_BOARD_H
