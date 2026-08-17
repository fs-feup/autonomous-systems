#include "motion_lib/brake_model/default_brake.hpp"

#include <algorithm>

common_lib::structures::Wheels DefaultBrake::calculate_brake_torques(double brake_demand) const {
  const double clamped_brake_demand = std::clamp(brake_demand, 0.0, 1.0);
  const auto& p = car_parameters_->brake_parameters;

  return common_lib::structures::Wheels(p->max_front_torque * clamped_brake_demand,
                                        p->max_front_torque * clamped_brake_demand,
                                        p->max_rear_torque * clamped_brake_demand,
                                        p->max_rear_torque * clamped_brake_demand);
}
