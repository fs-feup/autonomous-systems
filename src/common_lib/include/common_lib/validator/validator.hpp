#pragma once

#include <string>
#include <chrono>

namespace common_lib::validator {

class Validator {
private:
    double _expected_frequency_;
    std::chrono::time_point<std::chrono::system_clock> _last_timestamp; // Store the last timestamp

public:
    Validator(double expected_frequency);

    bool verifyFrequency();
};

} // namespace common_lib::validator
