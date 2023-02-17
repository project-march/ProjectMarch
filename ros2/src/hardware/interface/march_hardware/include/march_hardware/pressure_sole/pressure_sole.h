#ifndef MARCH_HARDWARE_PRESSURE_SOLE_CONTROLLER_H
#define MARCH_HARDWARE_PRESSURE_SOLE_CONTROLLER_H

#include <march_hardware/ethercat/slave.h>
#include <string>

namespace march {

namespace pressure_sole {
    inline constexpr static unsigned int DATA_LENGTH = 8;
}

struct PressureSoleData {
    double heel_right;
    double heel_left;
    double met1;
    double hallux;
    double met3;
    double toes;
    double met5;
    double arch;

    bool operator==(const PressureSoleData& rhs) const
    {
        return heel_right == rhs.heel_right && heel_left == rhs.heel_left && met1 == rhs.met1 && hallux == rhs.hallux
            && met3 == rhs.met3 && toes == rhs.toes && met5 == rhs.met5 && arch == rhs.arch;
    }

    /**
     * @brief Gives a pointer to the member variables so that the hardware interface can couple it back to the
     * broadcaster.
     * @attention Should contain exactly the same interface_names as expected by the
     * `march_pressure_sole_broadcaster/.../pressure_sole_semantic_component.hpp`
     * @returns A pair of interface_names and pointers to the member variables.
     */
    inline std::array<std::pair<std::string, double*>, pressure_sole::DATA_LENGTH> get_pointers()
    {
        return { std::make_pair("heel_right", &heel_right), std::make_pair("heel_left", &heel_left),
            std::make_pair("met1", &met1), std::make_pair("hallux", &hallux), std::make_pair("met3", &met3),
            std::make_pair("toes", &toes), std::make_pair("met5", &met5), std::make_pair("arch", &arch) };
    }

    /**
     * @brief Updates the member variables based on the input array.
     * @attention This 0 to 7 should be in the same order as it is send in the ethercat.
     * This method is used with the output from the read method of the `PressureSole`.
     * Make sure that the order, types (.ui or .f) and the offset for the read corresponds with position order seen via
     * the `ethercat-sdk`.
     * @param data The input array in order.
     */
    inline void update_values(std::array<bit32, pressure_sole::DATA_LENGTH> data)
    {
        heel_right = data[0].ui;
        heel_left = data[1].ui;
        met1 = data[2].ui;
        hallux = data[3].ui;
        met3 = data[4].ui;
        toes = data[5].ui;
        met5 = data[6].ui;
        arch = data[7].ui;
    }
};

class PressureSole : public Slave {
public:
    PressureSole(const Slave& slave, uint8_t byte_offset, std::string side);

    // Read the data in from ethercat
    PressureSoleData read();

    std::string getSide();

    constexpr static unsigned int PRESSURE_SOLE_DATA_LENGTH = 8;

private:
    const uint8_t byte_offset_;
    std::string side_;
};
} // namespace march

#endif // MARCH_HARDWARE_PRESSURE_SOLE_CONTROLLER_H
