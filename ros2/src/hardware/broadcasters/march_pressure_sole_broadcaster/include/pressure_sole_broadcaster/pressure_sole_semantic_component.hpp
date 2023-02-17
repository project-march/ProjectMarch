/// @author Marco Bak - M8

#ifndef BUILD_PRESSURE_SOLE_SEMANTIC_COMPONENT_HPP
#define BUILD_PRESSURE_SOLE_SEMANTIC_COMPONENT_HPP

#include <limits>
#include <string>
#include <vector>

#include "march_shared_msgs/msg/pressure_soles_data.hpp"
#include "semantic_components/semantic_component_interface.hpp"

namespace march_pressure_sole_broadcaster {
using StateInerfaces = std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>;
using PressureSoleMsg = march_shared_msgs::msg::PressureSoleData;

class PressureSoleSemanticComponent : public semantic_components::SemanticComponentInterface<PressureSoleMsg> {
public:
    explicit PressureSoleSemanticComponent()
        : SemanticComponentInterface("PressureSoles", /*size*/ 16)
    {
    }
};
} // namespace march_pressure_sole_broadcaster

#endif // BUILD_PRESSURE_SOLE_SEMANTIC_COMPONENT_HPP
