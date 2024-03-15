#include "include/inspection_functions.hpp"

InspectionFunctions::InspectionFunctions() {fromFile("src/inspection/src/config.txt");}

InspectionFunctions::InspectionFunctions(double maximum_angle, double ideal_velocity,
  double maximum_speed, double turning_time, double radius_of_wheel,
  double Gain, double timer, bool oscilate) {
    max_angle  = maximum_angle;
    ideal_speed = ideal_velocity;
    turning_period = turning_time;
    wheel_radius = radius_of_wheel;
    gain = Gain;
    max_speed = maximum_speed;
    finish_time = timer;
    oscilating = oscilate;
}

void InspectionFunctions::fromFile(const std::string &path) {
  std::string x, y;
  std::ifstream trackFile = openFileRead(path);
  while (trackFile >> x >> y) {
    if (x == "max_angle") {
        max_angle = stod(y);
    } else if (x == "ideal_speed") {
        ideal_speed = stod(y);
    } else if (x == "turning_period") {
        turning_period = stod(y);
    } else if (x == "wheel_radius") {
        wheel_radius = stod(y);
    } else if (x == "gain") {
        gain = stod(y);
    } else if (x == "max_speed") {
        max_speed = stod(y);
    } else if (x == "finish_time") {
        finish_time = stod(y);
    } else if (x == "oscilating") {
        if (y == "true") {
          oscilating = true;
        } else {
          oscilating = false;
        }
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

void InspectionFunctions::redefine_ideal_speed(double current_speed) {
  double velocity_error = abs(current_speed - ideal_speed);
  if (oscilating && velocity_error < 0.2) {
    if (ideal_speed == 0) {
      ideal_speed = max_speed;
    } else {
      ideal_speed = 0;
    }
  }
}