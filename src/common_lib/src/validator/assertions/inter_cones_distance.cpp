#include "common_lib/validator/assertions/inter_cones_distance.hpp"
#include <cmath>
#include <limits>
#include <algorithm>

namespace common_lib::validator::assertions {

bool InterConesDistance::validate(const custom_interfaces::msg::ConeArray& msg) const {
    if (msg.cone_array.empty()) {
        return true;
    }
    
    auto sortedCones = msg.cone_array;
    std::sort(sortedCones.begin(), sortedCones.end(), [](const auto& a, const auto& b) {
        return (a.position.x < b.position.x) || (a.position.x == b.position.x && a.position.y < b.position.y);
    });

    double min_distance = std::numeric_limits<double>::infinity();

    for (size_t i = 1; i < sortedCones.size(); ++i) {
        const auto& cone1 = sortedCones[i - 1];
        const auto& cone2 = sortedCones[i];
        double distance = std::sqrt(std::pow(cone1.position.x - cone2.position.x, 2) + std::pow(cone1.position.y - cone2.position.y, 2));
        if (distance < min_distance) {
            min_distance = distance;
        }
    }

    return !(min_distance < MIN_DISTANCE_ERROR);
}

std::string InterConesDistance::get_error_message() const {
    return "Minimum distance between cones not met.";
}

} // namespace common_lib::validator::assertions
