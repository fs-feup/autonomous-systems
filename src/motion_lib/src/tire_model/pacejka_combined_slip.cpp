#include "motion_lib/tire_model/pacejka_combined_slip.hpp"

std::pair<double, double> PacejkaCombinedSlip::tire_forces(double slip_angle, double slip_ratio,
                                                           double vertical_load) const {
  return {0, 0};
}