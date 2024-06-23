#include "common_lib/validator/assertions/planning/reasonable_curvature.hpp"

namespace common_lib::validator::assertions {

bool ReasonableCurvature::validate(const custom_interfaces::msg::PathPointArray& msg) const {
    for (size_t i = 0; i < msg.pathpoint_array.size() - 1; ++i) {
        const auto& point1 = msg.pathpoint_array[i];
        const auto& point2 = msg.pathpoint_array[i + 1];
        double scalar_product = point1.x * point2.x + point1.y * point2.y;
        double point1_size = std::sqrt(std::pow(point1.x, 2) + std::pow(point1.y, 2));
        double point2_size = std::sqrt(std::pow(point2.x, 2) + std::pow(point2.y, 2));
        double angle = std::acos(scalar_product / (point1_size * point2_size));
        if (angle * 180 / M_PI > MAX_CURVATURE_DEGREES) {
            return false;
        }
    }
    return true;
}

std::string ReasonableCurvature::get_error_message() const {
    return "Curvature is out of bounds.";
}

} // namespace common_lib::validator::assertions
