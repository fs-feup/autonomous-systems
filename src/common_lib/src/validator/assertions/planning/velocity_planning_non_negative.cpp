#include "common_lib/validator/assertions/planning/velocity_planning_non_negative.hpp"

namespace common_lib::validator::assertions {

bool VelocityPlanningNonNegative::validate(const custom_interfaces::msg::PathPointArray& msg) const {
    for (const auto& point : msg.pathpoint_array) {
        if (point.v < 0) {
            return false;
        }
    }
    return true;
}

std::string VelocityPlanningNonNegative::get_error_message() const {
    return "Velocity Planning cannot be negative.";
}

} // namespace common_lib::validator::assertions
