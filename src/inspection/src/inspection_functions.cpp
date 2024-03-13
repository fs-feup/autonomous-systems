#include "include/inspection_functions.hpp"

InspectionFunctions::InspectionFunctions() {fromFile("src/inspection/src/config.txt");}

InspectionFunctions::InspectionFunctions(double maximum_angle, double ideal_velocity,
  double turning_time, double radius_of_wheel, double Gain, double maximum_speed,
  bool EBS_multiple_times) {
    max_angle  = maximum_angle;
    ideal_speed = ideal_velocity;
    turning_period = turning_time;
    wheel_radius = radius_of_wheel;
    gain = Gain;
    max_speed = maximum_speed;
    EBS_multiple = EBS_multiple_times;
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
    } else if (x == "EBS_multiple") {
        if (y == "true") {
          EBS_multiple = true;
        } else {
          EBS_multiple = false;
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
