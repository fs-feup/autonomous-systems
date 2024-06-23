#pragma once

#include "common_lib/validator/assertions/assertion.hpp"
#include "custom_interfaces/msg/path_point_array.hpp"

namespace common_lib::validator::assertions {

/**
 * @brief Assertion to validate non-negative velocities in a path.
 * 
 * This class checks if all velocities in a `PathPointArray` message are non-negative.
 */
class VelocityPlanningNonNegative : public Assertion<custom_interfaces::msg::PathPointArray> {
public:
    bool validate(const custom_interfaces::msg::PathPointArray& msg) const override;
    std::string get_error_message() const override;
};

} // namespace common_lib::validator::assertions
