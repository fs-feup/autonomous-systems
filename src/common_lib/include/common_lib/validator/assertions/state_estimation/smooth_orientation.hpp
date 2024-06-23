#pragma once

#include "common_lib/validator/assertions/assertion.hpp"
#include "custom_interfaces/msg/vehicle_state.hpp"

#define MAX_ORIENTATION_DIFFERENCE_DEGREES 15

namespace common_lib::validator::assertions {

/**
 * @brief Assertion to validate smooth orientation change between vehicle states.
 * 
 * This class checks if the orientation difference between two consecutive `VehicleState`
 * messages does not exceed a defined maximum threshold.
 */
class SmoothOrientationAssert : public Assertion<custom_interfaces::msg::VehicleState> {
public:
    bool validate(const custom_interfaces::msg::VehicleState& msg1, const custom_interfaces::msg::VehicleState& msg2) const override;
    std::string get_error_message() const override;
};

} // namespace common_lib::validator::assertions
