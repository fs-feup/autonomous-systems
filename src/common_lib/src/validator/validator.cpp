#include "common_lib/validator/validator.hpp"

namespace common_lib::validator {

Validator::Validator(double expected_frequency)
    : _expected_frequency_(expected_frequency) {
    _last_timestamp = std::chrono::system_clock::now();
}

bool Validator::verifyFrequency() {
    auto current_time = std::chrono::system_clock::now();
    auto elapsed_seconds = std::chrono::duration_cast<std::chrono::seconds>(current_time - _last_timestamp).count();
    
    double actual_frequency = 1.0 / elapsed_seconds;

    _last_timestamp = current_time;

    return (_expected_frequency_ * 0.8) <= actual_frequency && actual_frequency <= (_expected_frequency_ * 1.2);
}

} // namespace common_lib::validator
