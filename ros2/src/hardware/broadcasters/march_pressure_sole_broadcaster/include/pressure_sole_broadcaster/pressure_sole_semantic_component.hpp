/// @author Marco Bak - M8

#ifndef BUILD_PRESSURE_SOLE_SEMANTIC_COMPONENT_HPP
#define BUILD_PRESSURE_SOLE_SEMANTIC_COMPONENT_HPP

#include <limits>
#include <string>
#include <vector>

#include "semantic_components/semantic_component_interface.hpp"
#include "march_shared_msgs/msg/pressure_soles_data.hpp"

namespace march_pressure_sole_broadcaster {
using StateInerfaces = std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>;
using PressureSoleMsg = march_shared_msgs::msg::PressureSoleData;

class PressureSoleSemanticComponent : public semantic_componensts::SemanticComponentInterface<PressureSoleMsg>{
public:
    explicit PressureSoleSemanticComponent()
        : SemanticComponentInterface("PressureSoles", /*size*/16)
    {
        
    }

};
}

#endif //BUILD_PRESSURE_SOLE_SEMANTIC_COMPONENT_HPP
