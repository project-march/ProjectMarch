#ifndef MARCH_HARDWARE_PRESSURE_SOLE_CONTROLLER_H
#define MARCH_HARDWARE_PRESSURE_SOLE_CONTROLLER_H

#include <march_hardware/ethercat/slave.h>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace march {

namespace pressure_sole {
    inline constexpr static unsigned int DATA_LENGTH = 2;
}

enum pressure_sole_side { left, right };

struct PressureSoleData {
    double heel_left;
    double heel_right = 2.0;
    // double met1;
    // double hallux;
    // double met3;
    // double toes;
    // double met5;
    // double arch;

    // pressure_sole_side side;

    // pressure_sole_side get_side()
    // {
    //     return side;
    // }

    friend bool operator==(const PressureSoleData& lhs, const PressureSoleData& rhs)
    {
        return lhs.heel_right == rhs.heel_right 
            && lhs.heel_left == rhs.heel_left ;
            // && lhs.met1 == rhs.met1 
            // && lhs.hallux == rhs.hallux
            // && lhs.met3 == rhs.met3 
            // && lhs.toes == rhs.toes 
            // && lhs.met5 == rhs.met5 
            // && lhs.arch == rhs.arch;
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
        RCLCPP_INFO(rclcpp::get_logger("get_pointers_logger"), "heel_right %f", heel_right);
        return { 
            std::make_pair("heel_right", &heel_right), std::make_pair("heel_left", &heel_left),
            // std::make_pair("met1", &met1), std::make_pair("hallux", &hallux), std::make_pair("met3", &met3),
            // std::make_pair("toes", &toes), std::make_pair("met5", &met5), std::make_pair("arch", &arch) 
            };
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
        heel_right = data[0].f;
        heel_left = data[1].f;
        // met1 = data[2].f;
        // hallux = data[3].f;
        // met3 = data[4].f;
        // toes = data[5].f;
        // met5 = data[6].f;
        // arch = data[7].f;
        RCLCPP_INFO(rclcpp::get_logger("update_values_logger"), "heel_right %f", heel_right);
        // get_pointers();
    }
};

class PressureSole : public Slave {
public:
    PressureSole(const Slave& slave, uint8_t byte_offset, std::string side);

    // Read the data in from ethercat
    void read(PressureSoleData& pressure_sole_data) const;

    std::string getSide();

    constexpr static unsigned int PRESSURE_SOLE_DATA_LENGTH = 8;

private:
    const uint8_t byte_offset_;
    std::string side_;
};
} // namespace march

#endif // MARCH_HARDWARE_PRESSURE_SOLE_CONTROLLER_H
