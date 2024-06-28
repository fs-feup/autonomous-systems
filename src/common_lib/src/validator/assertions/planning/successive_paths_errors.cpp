#include "common_lib/validator/assertions/planning/successive_paths_errors.hpp"
#include <cmath>

namespace common_lib::validator::assertions {

bool SuccessivePathsErrors::validate(const custom_interfaces::msg::PathPointArray& msg1,
                                     const custom_interfaces::msg::PathPointArray& msg2) const {
    if (msg1.pathpoint_array.empty() || msg2.pathpoint_array.empty()) {
        return true;
    }

    const auto& start_point2 = msg2.pathpoint_array.front();
    double min_distance = std::numeric_limits<double>::max();
    size_t closest_index = 0;

    for (size_t i = 0; i < msg1.pathpoint_array.size(); ++i) {
        const auto& point1 = msg1.pathpoint_array[i];
        double distance = std::sqrt(std::pow(point1.x - start_point2.x, 2) + std::pow(point1.y - start_point2.y, 2));
        if (distance < min_distance) {
            min_distance = distance;
            closest_index = i;
        }
    }

    double total_error = 0.0;
    size_t count = 0;

    for (size_t i = closest_index; i < std::min(msg1.pathpoint_array.size(), msg2.pathpoint_array.size()); ++i) {

        const auto& point1 = msg1.pathpoint_array[i];
        const auto& point2 = msg2.pathpoint_array[i - closest_index];

        double error = (point1.x - point2.x) + (point1.y - point2.y) + (point1.v - point2.v);
        total_error += error;
        ++count;
    }

    double average_error = total_error / static_cast<double>(count);

    return average_error <= MAX_ERROR;
}

std::string SuccessivePathsErrors::get_error_message() const {
    return "Mean Error Out of Bounds";
}

} // namespace common_lib::validator::assertions