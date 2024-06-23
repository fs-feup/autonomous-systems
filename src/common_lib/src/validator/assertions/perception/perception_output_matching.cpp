#include "common_lib/validator/assertions/perception/perception_output_matching.hpp"
#include <cmath>
#include <limits>
#include <algorithm>

namespace common_lib::validator::assertions {

bool PerceptionOutputMatching::validate(const custom_interfaces::msg::ConeArray& msg1, const custom_interfaces::msg::ConeArray& msg2) const {
    if (msg1.cone_array.empty() || msg2.cone_array.empty()) {
        return true;
    }

    double sum_error = 0.0;
    for (size_t i = 0; i < std::min(msg1.cone_array.size(), msg2.cone_array.size()); ++i) {
        double min_distance = std::numeric_limits<double>::infinity();
        for (size_t j = 0; j < std::min(msg1.cone_array.size(), msg2.cone_array.size()); ++j) {
            const auto& cone1 = msg1.cone_array[i];
            const auto& cone2 = msg2.cone_array[j];
            double distance = std::sqrt(std::pow(cone1.position.x - cone2.position.x, 2) + std::pow(cone1.position.y - cone2.position.y, 2));
            if (distance < min_distance) {
                min_distance = distance;
            }
        }
        sum_error += min_distance;
    }

    double average_error = sum_error / std::min(msg1.cone_array.size(), msg2.cone_array.size());

    return average_error < MAX_ERROR;
}

std::string PerceptionOutputMatching::get_error_message() const {
    return "Average distance between corresponding cones exceeds the maximum allowed error.";
}

} // namespace common_lib::validator::assertions
