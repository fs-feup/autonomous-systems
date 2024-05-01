#include "include/inspection_functions.hpp"

InspectionFunctions::InspectionFunctions(double max_angle, double turning_period,
                                         double wheel_radius, double finish_time,
                                         bool start_and_stop, double gain /* = 0.0 */,
                                         double ideal_velocity /* = 0.0 */) {
  this->max_angle = max_angle;
  this->ideal_velocity = ideal_velocity;
  this->turning_period = turning_period;
  this->wheel_radius = wheel_radius;
  this->gain = gain;
  this->finish_time = finish_time;
  this->start_and_stop = start_and_stop;
  current_goal_velocity = ideal_velocity;
}

InspectionFunctions::InspectionFunctions() = default;

double InspectionFunctions::rpm_to_velocity(double rpm) {
  double perimeter = 2 * M_PI * wheel_radius;
  return rpm * perimeter / 60.0;
}

double InspectionFunctions::calculate_throttle(double current_velocity) {
  double error = current_goal_velocity - current_velocity;
  return gain * error;
}

double InspectionFunctions::calculate_steering(double time) {
  return sin((time * 2 * M_PI / turning_period)) * max_angle;
}

double InspectionFunctions::throttle_to_adequate_range(double throttle) {
  if (fabs(throttle) >= VELOCITY_ERROR_FOR_MAX_THROTTLE) {
    return (throttle > 0) ? 1 : -1;
  }
  return throttle/VELOCITY_ERROR_FOR_MAX_THROTTLE;
}

void InspectionFunctions::redefine_goal_velocity(double current_velocity) {
  if (start_and_stop && abs(current_velocity - this->current_goal_velocity) < 0.2) {
    if (this->current_goal_velocity == 0) {
      this->current_goal_velocity = this->ideal_velocity;
    } else {
      this->current_goal_velocity = 0;
    }
  }
  if (!start_and_stop) {
    this->current_goal_velocity = 0;
  }
}