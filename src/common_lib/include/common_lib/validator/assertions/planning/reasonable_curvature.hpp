#pragma once

#include "common_lib/validator/assertions/assertion.hpp"
#include "custom_interfaces/msg/path_point_array.hpp"

#define MAX_CURVATURE_DEGREES 30

namespace common_lib::validator::assertions {

/**
 * @brief Assertion to validate reasonable curvature in a path.
 * 
 * This class checks if the curvature between consecutive points in a `PathPointArray`
 * message does not exceed a defined maximum threshold.
 */
class ReasonableCurvature : public Assertion<custom_interfaces::msg::PathPointArray> {
public:
    bool validate(const custom_interfaces::msg::PathPointArray& msg) const override;
    std::string get_error_message() const override;
};

} // namespace common_lib::validator::assertions
