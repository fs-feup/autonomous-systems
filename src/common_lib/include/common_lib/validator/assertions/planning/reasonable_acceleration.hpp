#pragma once

#include "common_lib/validator/assertions/assertion.hpp"
#include "custom_interfaces/msg/path_point_array.hpp"

#define MAX_ACCELERATION 10

namespace common_lib::validator::assertions {

class ReasonableAcceleration : public Assertion<custom_interfaces::msg::PathPointArray> {
public:
    bool validate(const custom_interfaces::msg::PathPointArray& msg) const override;
    std::string get_error_message() const override;
};

} // namespace common_lib::validator::assertions
