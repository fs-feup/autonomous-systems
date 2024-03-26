#ifndef INVALID_MISSION_EXCEPTION_HPP
#define INVALID_MISSION_EXCEPTION_HPP

#include <exception>
#include <string>

class InvalidMissionException : public std::exception {
  std::string message;

 public:
  explicit InvalidMissionException(std::string msg) : message(msg) {}

  const char *what() const throw() {
    return ("Invalid mission. Please choose between 'inspection' and 'inspection_test_EBS'; chose "
            "'" +
            message + "' instead.")
        .c_str();
  }
};

#endif  // INVALID_MISSION_EXCEPTION_HPP