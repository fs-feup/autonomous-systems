#include "rclcpp/rclcpp.hpp"
#include "functions/inspection_functions.hpp"

InspectionFunctions::InspectionFunctions(double max_angle, double turning_period,
                                         double wheel_radius, double finish_time,
                                         bool start_and_stop, double gain /* = 0.0 */,
                                         double ideal_velocity /* = 0.0 */)
    : max_angle_(max_angle),
      ideal_velocity_(ideal_velocity),
      turning_period_(turning_period),
      wheel_radius_(wheel_radius),
      gain_(gain),
      finish_time_(finish_time),
      current_goal_velocity_(ideal_velocity),
      start_and_stop_(start_and_stop) {}

InspectionFunctions::InspectionFunctions() = default;

double InspectionFunctions::rpm_to_velocity(double rpm) const {
  double perimeter = 2 * M_PI * wheel_radius_;
  return rpm * perimeter / 60.0;
}

double InspectionFunctions::calculate_throttle(double current_velocity) const {
  RCLCPP_DEBUG(rclcpp::get_logger("inspection"), "Current goal velocity - current velocity: %lf-%lf", current_goal_velocity_, current_velocity);
  double error = current_goal_velocity_ - current_velocity;
  return gain_ * error;
}

double InspectionFunctions::calculate_steering(double time) const {
  return stop_oscilating_ ? 0.0 : sin(time * 2 * M_PI / turning_period_) * max_angle_;
}

double InspectionFunctions::throttle_to_adequate_range(double throttle) const {
  if (fabs(throttle) >= MAX_THROTTLE) {
    return (throttle > 0) ? 1 : -1;
  }
  return throttle / MAX_THROTTLE;
}

void InspectionFunctions::redefine_goal_velocity(double current_velocity) {
  if (start_and_stop_ && fabs(current_velocity - this->current_goal_velocity_) < 0.1) {
    if (this->current_goal_velocity_ == 0) {
      this->current_goal_velocity_ = this->ideal_velocity_;
    } else {
      this->current_goal_velocity_ = 0;
    }
  }
}