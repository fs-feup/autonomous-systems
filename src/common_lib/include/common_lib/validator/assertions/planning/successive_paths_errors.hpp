#pragma once

#include "common_lib/validator/assertions/assertion.hpp"
#include "custom_interfaces/msg/path_point_array.hpp"

#define MAX_ERROR 0

namespace common_lib::validator::assertions {

/**
 * @brief Assertion to validate errors between successive paths.
 * 
 * This class checks if the error between corresponding points in two `PathPointArray`
 * messages is within a defined maximum threshold.
 */
class SuccessivePathsErrors : public Assertion<custom_interfaces::msg::PathPointArray> {
public:
    bool validate(const custom_interfaces::msg::PathPointArray& msg1, const custom_interfaces::msg::PathPointArray& msg2) const override;
    std::string get_error_message() const override;
};

} // namespace common_lib::validator::assertions
