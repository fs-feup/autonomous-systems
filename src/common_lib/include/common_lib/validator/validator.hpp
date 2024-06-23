#pragma once

#include <string>
#include <chrono>
#include "common_lib/validator/assertions/assertion.hpp"
#include <vector>
#include <memory>

namespace common_lib::validator {

template<typename T>
class Validator {
private:
    double _expected_frequency_;
    std::chrono::time_point<std::chrono::system_clock> _last_timestamp;
    std::vector<std::shared_ptr<assertions::Assertion<T>>> _assertions_;
    T _last_message_;

public:
    Validator(double expected_frequency, std::vector<std::shared_ptr<assertions::Assertion<T>>> assertions);

    bool verify(T msg, std::vector<std::string>& reports);

    bool verify_frequency();
};

} // namespace common_lib::validator
