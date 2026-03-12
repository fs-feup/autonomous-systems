#include "motion_lib/tire_model/pacejka_combined_slip.hpp"

Eigen::Vector3d PacejkaCombinedSlip::tire_forces(const TireInput& tire_input)  {
  // Longitudinal pure slip force (Fx0)
  double Bx = this->car_parameters_->tire_parameters->tire_B_longitudinal;
  double Cx = this->car_parameters_->tire_parameters->tire_C_longitudinal;
  double Dx = this->car_parameters_->tire_parameters->tire_D_longitudinal * tire_input.vertical_load;
  double Ex = this->car_parameters_->tire_parameters->tire_E_longitudinal;

  double Fx0 = Dx * std::sin(Cx * std::atan(Bx * tire_input.slip_ratio -
                                            Ex * (Bx * tire_input.slip_ratio - std::atan(Bx * tire_input.slip_ratio))));

  // Lateral pure slip force (Fy0)
  double By = this->car_parameters_->tire_parameters->tire_B_lateral;
  double Cy = this->car_parameters_->tire_parameters->tire_C_lateral;
  double Dy = this->car_parameters_->tire_parameters->tire_D_lateral * tire_input.vertical_load;
  double Ey = this->car_parameters_->tire_parameters->tire_E_lateral;
  double Fy0 = Dy * std::sin(Cy * std::atan(By * tire_input.slip_angle -
                                            Ey * (By * tire_input.slip_angle - std::atan(By * tire_input.slip_angle))));

  // Combined slip reduction factors (friction ellipse type)
  double Gx = std::cos(std::atan(By * tire_input.slip_angle));
  double Gy = std::cos(std::atan(Bx * tire_input.slip_ratio));

  // Final forces with combined slip
  double Fx = Fx0 * Gx;
  double Fy = Fy0 * Gy;
  
  // This model ignores the self aligning moment
  return Eigen::Vector3d(Fx, Fy, 0.0);
}