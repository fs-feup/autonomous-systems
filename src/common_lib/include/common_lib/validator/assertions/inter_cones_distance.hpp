#pragma once

#include "common_lib/validator/assertions/assertion.hpp"
#include "custom_interfaces/msg/cone_array.hpp"

#define MIN_DISTANCE_ERROR 0.3

namespace common_lib::validator::assertions {

/**
 * @brief Assertion to validate the distance between cones.
 * 
 * This class checks if the distance between any two cones in a `ConeArray` message
 * is greater than a defined minimum distance.
 */
class InterConesDistance : public Assertion<custom_interfaces::msg::ConeArray> {
public:
    bool validate(const custom_interfaces::msg::ConeArray& msg) const override;
    std::string get_error_message() const override;
};

} // namespace common_lib::validator::assertions
