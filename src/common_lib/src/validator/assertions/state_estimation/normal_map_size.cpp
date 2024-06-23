#include "common_lib/validator/assertions/state_estimation/normal_map_size.hpp"

namespace common_lib::validator::assertions {

bool NormalMapSizeAssertion::validate(const custom_interfaces::msg::ConeArray& msg) const {
    return msg.cone_array.size() < 250;
}

std::string NormalMapSizeAssertion::get_error_message() const {
    return "Map Size very high!";
}

} // namespace common_lib::validator::assertions
