#pragma once

#include <string>
#include <chrono>
#include "common_lib/validator/assertions/assertion.hpp"
#include <vector>
#include <memory>

namespace common_lib::validator {

/**
 * @brief A class for validating messages based on assertions and expected frequency.
 * 
 * This template class validates messages of type T using a set of assertions and ensures
 * that messages are received at an expected frequency.
 * 
 * @tparam T The type of the message to be validated.
 */
template<typename T>
class Validator {
private:
    double _expected_frequency_; ///< The expected frequency of the messages.
    std::chrono::time_point<std::chrono::system_clock> _last_timestamp; ///< The timestamp of the last validated message.
    std::vector<std::shared_ptr<assertions::Assertion<T>>> _assertions_; ///< A vector of assertions used for validation.
    T _last_message_; ///< The last message.

public:
    /**
     * @brief Constructs a new Validator object.
     * 
     * @param expected_frequency The expected frequency of the messages.
     * @param assertions A vector of assertions used to validate the messages.
     */
    Validator(double expected_frequency, std::vector<std::shared_ptr<assertions::Assertion<T>>> assertions);

    /**
     * @brief Verifies a message against the assertions.
     * 
     * This function checks if the provided message meets all the assertion criteria.
     * 
     * @param msg The message to be validated.
     * @param reports A vector to store the error messages if validation fails.
     * @return true if the message is valid, false otherwise.
     */
    bool verify(T msg, std::vector<std::string>& reports);
    
    /**
     * @brief Verifies the frequency of the messages.
     * 
     * This function checks if the messages are being received at the expected frequency.
     * 
     * @return true if the messages are received at the expected frequency, false otherwise.
     */
    bool verify_frequency();
};

} // namespace common_lib::validator
