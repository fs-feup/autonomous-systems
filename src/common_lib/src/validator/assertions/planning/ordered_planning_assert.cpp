#include "common_lib/validator/assertions/planning/ordered_planning_assert.hpp"

namespace common_lib::validator::assertions {

bool OrderedPlanningAssertion::validate(const custom_interfaces::msg::PathPointArray& msg) const {
    for (size_t i = 0; i < msg.pathpoint_array.size() - 1; ++i) {
        const auto& point1 = msg.pathpoint_array[i];
        const auto& point2 = msg.pathpoint_array[i + 1];
        double scalar_product = point1.x * point2.x + point1.y * point2.y;
        if (scalar_product < 0) {
            return false;
        }
    }
    return true;
}

std::string OrderedPlanningAssertion::get_error_message() const {
    return "Path points are not ordered correctly.";
}

} // namespace common_lib::validator::assertions
