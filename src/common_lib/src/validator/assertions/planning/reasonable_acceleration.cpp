#include "common_lib/validator/assertions/planning/reasonable_acceleration.hpp"

namespace common_lib::validator::assertions {

bool ReasonableAcceleration::validate(const custom_interfaces::msg::PathPointArray& msg) const {
    for (size_t i = 0; i < msg.pathpoint_array.size() - 1; ++i) {
        const auto& point1 = msg.pathpoint_array[i];
        const auto& point2 = msg.pathpoint_array[i + 1];
        double delta_x = std::sqrt(std::pow(point1.x - point2.x, 2) + std::pow(point1.y * point2.y, 2));
        double vel_ini = point1.v;
        double vel_fin = point2.v;
        double acceleration = (std::pow(vel_fin, 2) - std::pow(vel_ini, 2)) / (2 * delta_x);
        if (std::abs(acceleration) > MAX_ACCELERATION) {
            return false;
        }
    }
    return true;
}

std::string ReasonableAcceleration::get_error_message() const {
    return "Acceleration is out of bounds.";
}

} // namespace common_lib::validator::assertions
