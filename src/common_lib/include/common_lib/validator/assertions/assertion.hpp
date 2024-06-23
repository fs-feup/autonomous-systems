#pragma once

#include <string>
#include <cmath>
#include <limits>
#include <algorithm>

namespace common_lib::validator::assertions {

template<typename T>
class Assertion {
public:
    virtual bool validate(const T& msg) const {
        return true;
    }
    
    virtual bool validate(const T& msg1, const T& msg2) const {
        return true;
    }
    
    virtual std::string get_error_message() const = 0;
};

}
