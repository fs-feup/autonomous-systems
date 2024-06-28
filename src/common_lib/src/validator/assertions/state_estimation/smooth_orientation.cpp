#include "common_lib/validator/assertions/state_estimation/smooth_orientation.hpp"

namespace common_lib::validator::assertions {

bool SmoothOrientationAssert::validate(const custom_interfaces::msg::VehicleState& msg1, const custom_interfaces::msg::VehicleState& msg2) const {
    return std::abs(msg1.theta - msg2.theta) < MAX_ORIENTATION_DIFFERENCE_DEGREES
}

std::string SmoothOrientationAssert::get_error_message() const {
    return "Orientation difference very high";
}

} // namespace common_lib::validator::assertions
