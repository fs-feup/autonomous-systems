#include "functions/inspection_functions.hpp"

InspectionFunctions::InspectionFunctions(double max_angle, double turning_period,
                                         double wheel_radius, double finish_time,
                                         bool start_and_stop, double gain /* = 0.0 */,
                                         double ideal_velocity /* = 0.0 */)
    : max_angle(max_angle),
      ideal_velocity(ideal_velocity),
      turning_period(turning_period),
      wheel_radius(wheel_radius),
      gain(gain),
      finish_time(finish_time),
      current_goal_velocity(ideal_velocity),
      start_and_stop(start_and_stop) {}

InspectionFunctions::InspectionFunctions() = default;

double InspectionFunctions::rpm_to_velocity(double rpm) const {
  double perimeter = 2 * M_PI * wheel_radius;
  return rpm * perimeter / 60.0;
}

double InspectionFunctions::calculate_throttle(double current_velocity) const {
  double error = current_goal_velocity - current_velocity;
  return gain * error;
}

double InspectionFunctions::calculate_steering(double time) const {
  return sin(time * 2 * M_PI / turning_period) * max_angle;
}

double InspectionFunctions::throttle_to_adequate_range(double throttle) const {
  if (fabs(throttle) >= MAX_THROTTLE) {
    return (throttle > 0) ? 1 : -1;
  }
  return throttle / MAX_THROTTLE;
}

void InspectionFunctions::redefine_goal_velocity(double current_velocity) {
  if (start_and_stop && fabs(current_velocity - this->current_goal_velocity) < 0.2) {
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