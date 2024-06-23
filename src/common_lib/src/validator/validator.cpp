#include "common_lib/validator/validator.hpp"

namespace common_lib::validator {

template<typename T>
Validator<T>::Validator(double expected_frequency, std::vector<std::shared_ptr<assertions::Assertion<T>>> assertions)
    : _expected_frequency_(expected_frequency), _assertions_(std::move(assertions)) {
    _last_timestamp = std::chrono::system_clock::now();
}

template<typename T>
bool Validator<T>::verify_frequency() {
    auto current_time = std::chrono::system_clock::now();
    auto elapsed_seconds = std::chrono::duration_cast<std::chrono::seconds>(current_time - _last_timestamp).count();
    
    double actual_frequency = 1.0 / elapsed_seconds;

    _last_timestamp = current_time;

    return (_expected_frequency_ * 0.8) <= actual_frequency && actual_frequency <= (_expected_frequency_ * 1.2);
}

template<typename T>
bool Validator<T>::verify(T msg, std::vector<std::string>& reports) {
    bool all_valid = true;

    if (!verify_frequency()){
        all_valid = false;
        reports.push_back("Frequency different than expected");
    }

    for (const auto& assertion : _assertions_) {
        if (!assertion->validate(msg)) {
            all_valid = false;
            reports.push_back(assertion->get_error_message());
        }
        if (!assertion->validate(msg, _last_message_)){
            all_valid = false;
            reports.push_back(assertion->get_error_message());
        }
    }
    _last_message_ = msg;
    return all_valid;
}

} // namespace common_lib::validator
