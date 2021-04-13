#ifndef MARCH_HARDWARE_PRESSURE_SOLE_CONTROLLER_H
#define MARCH_HARDWARE_PRESSURE_SOLE_CONTROLLER_H

#include <march_hardware/ethercat/slave.h>
#include <string>

namespace march {
struct PressureSoleData {
    float heel_right;
    float heel_left;
    float met1;
    float hallux;
    float met3;
    float toes;
    float met5;
    float arch;

    bool operator==(const PressureSoleData& rhs) const
    {
        return heel_right == rhs.heel_right && heel_left == rhs.heel_left
            && met1 == rhs.met1 && hallux == rhs.hallux && met3 == rhs.met3
            && toes == rhs.toes && met5 == rhs.met5 && arch == rhs.arch;
    }
};

class PressureSole : public Slave {
public:
    PressureSole(const Slave& slave, uint8_t byte_offset, std::string side);

    // Read the data in from ethercat
    PressureSoleData read();

    std::string getSide();

    const unsigned int PRESSURE_SOLE_DATA_LENGTH = 8;

private:
    const uint8_t byte_offset_;
    std::string side_;
};
} // namespace march

#endif // MARCH_HARDWARE_PRESSURE_SOLE_CONTROLLER_H
