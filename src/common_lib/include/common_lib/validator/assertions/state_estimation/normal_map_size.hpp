#pragma once

#include "common_lib/validator/assertions/assertion.hpp"
#include "custom_interfaces/msg/cone_array.hpp"

namespace common_lib::validator::assertions {

/**
 * @brief Assertion to validate normal map size in a ConeArray message.
 * 
 * This class checks if the size of the normal map in a `ConeArray` message meets certain criteria.
 */
class NormalMapSizeAssertion : public Assertion<custom_interfaces::msg::ConeArray> {
public:
    bool validate(const custom_interfaces::msg::ConeArray& msg) const override;
    std::string get_error_message() const override;
};

} // namespace common_lib::validator::assertions
