#include "include/inspection_functions.hpp"

InspectionFunctions::InspectionFunctions() {define_constants("src/inspection/src/config.txt");}

void InspectionFunctions::define_constants(const std::string &path) {
  std::string x, y;
  std::ifstream trackFile = openFileRead(path);
  while (trackFile >> x >> y) {
    double yValue = stod(y);
    if (x == "max_angle") {
        max_angle = yValue;
    } else if (x == "ideal_speed") {
        ideal_speed = yValue;
    } else if (x == "turning_period") {
        turning_period = yValue;
    } else if (x == "wheel_radius") {
        wheel_radius = yValue;
    } else if (x == "gain") {
        gain = yValue;
    } else if (x == "max_speed") {
        max_speed = yValue;
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
      "Invalid variable %s when defining constants for inspection", x.c_str());
    }
  }
  trackFile.close();
}

double InspectionFunctions::rpm_to_speed(double rpm) {
  double perimeter = 2*M_PI*wheel_radius;
  return rpm*perimeter/60.0;
}

double InspectionFunctions::calculate_torque(double velocity) {
  double error = ideal_speed - velocity;
  return gain*error;
}

double InspectionFunctions::calculate_steering(double time) {
  return sin((time*2*M_PI/turning_period))*max_angle;
}

