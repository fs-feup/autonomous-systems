#ifndef INVALID_MISSION_EXCEPTION_HPP
#define INVALID_MISSION_EXCEPTION_HPP

#include <exception>
#include <string>

class InvalidMissionException : public std::exception {
  std::string message;

 public:
  explicit InvalidMissionException(const std::string& msg) : message(msg) {
    this->message =
        "Invalid mission. Please choose between 'inspection' and 'inspection_test_EBS'; chose '" +
        msg + "' instead.";
  }

  const char* what() const throw() { return this->message.c_str(); }
};

#endif  // INVALID_MISSION_EXCEPTION_HPP