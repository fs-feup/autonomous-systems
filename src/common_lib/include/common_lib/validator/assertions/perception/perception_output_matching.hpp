#pragma once

#include "common_lib/validator/assertions/assertion.hpp"
#include "custom_interfaces/msg/cone_array.hpp"

#define MAX_ERROR 0.3

namespace common_lib::validator::assertions {

class PerceptionOutputMatching : public Assertion<custom_interfaces::msg::ConeArray> {
public:
    bool validate(const custom_interfaces::msg::ConeArray& msg1, const custom_interfaces::msg::ConeArray& msg2) const override;
    std::string get_error_message() const override;
};

} // namespace common_lib::validator::assertions
