#pragma once

#include <string>
#include <cmath>
#include <limits>
#include <algorithm>

namespace common_lib::validator::assertions {

/**
 * @brief Abstract base class for assertions.
 * 
 * This template class defines the interface for assertions used to validate messages of type T.
 * 
 * @tparam T The type of the message to be validated.
 */
template<typename T>
class Assertion {
public:
    /**
     * @brief Validates a single message.
     * 
     * This function checks if the provided message meets the assertion criteria.
     * The default implementation always returns true.
     * 
     * @param msg The message to be validated.
     * @return true if the message is valid, false otherwise.
     */
    virtual bool validate(const T& msg) const {
        return true;
    }
    
    /**
     * @brief Validates a pair of messages.
     * 
     * This function checks if the provided pair of messages meet the assertion criteria.
     * The default implementation always returns true.
     * 
     * @param msg1 The most recent message.
     * @param msg2 The second most recent message.
     * @return true if the messages are valid, false otherwise.
     */
    virtual bool validate(const T& msg1, const T& msg2) const {
        return true;
    }
    
    /**
     * @brief Gets the error message.
     * 
     * This function returns the error message associated with a failed validation.
     * 
     * @return A string containing the error message.
     */
    virtual std::string get_error_message() const = 0;
};

}
