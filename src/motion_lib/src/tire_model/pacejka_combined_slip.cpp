#include "motion_lib/tire_model/pacejka_combined_slip.hpp"

std::pair<double, double> PacejkaCombinedSlip::tire_forces(double slip_angle, double slip_ratio,
                                                           double vertical_load) const {
  // Longitudinal pure slip force (Fx0)
  double Bx = this->car_parameters_->tire_parameters.tire_B_longitudinal;
  double Cx = this->car_parameters_->tire_parameters.tire_C_longitudinal;
  double Dx = this->car_parameters_->tire_parameters.tire_D_longitudinal * vertical_load;
  double Ex = this->car_parameters_->tire_parameters.tire_E_longitudinal;

  double Fx0 =
      Dx * std::sin(Cx * std::atan(Bx * slip_ratio - Ex * (Bx * slip_ratio - std::atan(Bx * slip_ratio))));

  // Lateral pure slip force (Fy0)
  double By = this->car_parameters_->tire_parameters.tire_B_lateral;
  double Cy = this->car_parameters_->tire_parameters.tire_C_lateral;
  double Dy = this->car_parameters_->tire_parameters.tire_D_lateral * vertical_load;
  double Ey = this->car_parameters_->tire_parameters.tire_E_lateral;

  double Fy0 =
      Dy * std::sin(Cy * std::atan(By * slip_angle - Ey * (By * slip_angle - std::atan(By * slip_angle))));

  // Combined slip reduction factors (friction ellipse type)
  double Gx = std::cos(std::atan(By * slip_angle));
  double Gy = std::cos(std::atan(Bx * slip_ratio));

  // Final forces with combined slip
  double Fx = Fx0 * Gx;
  double Fy = Fy0 * Gy;

  return std::make_pair(Fx, Fy);
}