#include "motion_lib/tire_model/base_tire_model.hpp"
#include <algorithm>
#include <cmath>

// Common
Eigen::Vector2d calculate_contact_patch_velocity(const TireInput& tire_input, double track_width) {
  const bool is_front = tire_input.tire == FL || tire_input.tire == FR;
  const bool is_left = tire_input.tire == FL || tire_input.tire == RL;

  const double wheel_position_x = is_front ? tire_input.distance_to_CG : -tire_input.distance_to_CG;
  const double wheel_position_y = is_left ? track_width / 2.0 : -track_width / 2.0;

  const double vehicle_x = tire_input.vx - tire_input.yaw_rate * wheel_position_y;
  const double vehicle_y = tire_input.vy + tire_input.yaw_rate * wheel_position_x;

  const double cos_delta = std::cos(tire_input.steering_angle);
  const double sin_delta = std::sin(tire_input.steering_angle);

  return Eigen::Vector2d(vehicle_x * cos_delta + vehicle_y * sin_delta,
                         -vehicle_x * sin_delta + vehicle_y * cos_delta);
}

// Not transient

void TireModel::calculate_slip_angle_front_not_transient(TireInput& tire_input) {
  const double V_eps = 0.5;
  const double half_track = car_parameters_->track_width / 2.0;

  double y_sign = (tire_input.tire == FL) ? 1.0 : -1.0;
  double lf = car_parameters_->wheelbase - car_parameters_->cg_2_rear_axis;

  double vx_hub = tire_input.vx - (tire_input.yaw_rate * y_sign * half_track);
  double vy_hub = tire_input.vy + (tire_input.yaw_rate * lf);

  // Rotate into the steered wheel frame.
  double Vcx = vx_hub * cos(tire_input.steering_angle) + vy_hub * sin(tire_input.steering_angle);
  double Vcy = -vx_hub * sin(tire_input.steering_angle) + vy_hub * cos(tire_input.steering_angle);

  double Vlong_reg = std::sqrt(Vcx * Vcx + V_eps * V_eps);
  double direction = Vcx / Vlong_reg;
  tire_input.slip_angle = atan2(Vcy, Vlong_reg) * direction;

  tire_input.last_slip_angle[tire_input.tire] = tire_input.slip_angle;
}

void TireModel::calculate_slip_angle_rear_not_transient(TireInput& tire_input) {
  const double V_eps = 0.5;
  const double half_track = car_parameters_->track_width / 2.0;

  double y_sign = (tire_input.tire == RL) ? 1.0 : -1.0;
  double lr = car_parameters_->cg_2_rear_axis;

  double vx_hub = tire_input.vx - (tire_input.yaw_rate * y_sign * half_track);
  double vy_hub = tire_input.vy - (tire_input.yaw_rate * lr);

  double Vlong_reg = std::sqrt(vx_hub * vx_hub + V_eps * V_eps);
  double direction = vx_hub / Vlong_reg;
  tire_input.slip_angle = atan2(vy_hub, Vlong_reg) * direction;

  tire_input.last_slip_angle[tire_input.tire] = tire_input.slip_angle;
}


void TireModel::calculate_slip_ratio_not_transient(TireInput& tire_input) {
  const double half_track = car_parameters_->track_width / 2.0;

  bool is_rear = (tire_input.tire == RL || tire_input.tire == RR);
  double l_axle = is_rear ? car_parameters_->cg_2_rear_axis
                          : (car_parameters_->wheelbase - car_parameters_->cg_2_rear_axis);

  double y_sign = (tire_input.tire == FL || tire_input.tire == RL) ? 1.0 : -1.0;
  double x_sign = is_rear ? -1.0 : 1.0;

  double vx_hub = tire_input.vx - (tire_input.yaw_rate * y_sign * half_track);
  double vy_hub = tire_input.vy + (tire_input.yaw_rate * x_sign * l_axle);

  double Vcx = vx_hub * std::cos(tire_input.steering_angle) + vy_hub * std::sin(tire_input.steering_angle);
  double Vw = tire_input.wheel_angular_speed * car_parameters_->tire_parameters->effective_tire_r;

  const double V_floor = 1.0;
  double denom = std::max({std::abs(Vw), std::abs(Vcx), V_floor});
  double slip_target = std::clamp((Vw - Vcx) / denom, -1.0, 1.0);

  tire_input.slip_ratio = slip_target;
  tire_input.last_slip_ratio[tire_input.tire] = slip_target;
}

Eigen::Vector4d TireModel::calculate_tire_forces_not_transient(TireInput& tire_input) {
  if (tire_input.tire == FL || tire_input.tire == FR) {
    if (tire_input.tire == FL) {
      tire_input.distance_to_CG = car_parameters_->tire_parameters->d_fleft;
      tire_input.camber_angle = car_parameters_->tire_parameters->fl_camber;
    } else {
      tire_input.distance_to_CG = car_parameters_->tire_parameters->d_fright;
      tire_input.camber_angle = car_parameters_->tire_parameters->fr_camber;
    }

    calculate_slip_angle_front_not_transient(tire_input);
  } else {
    if (tire_input.tire == RL) {
      tire_input.distance_to_CG = car_parameters_->tire_parameters->d_bleft;
      tire_input.camber_angle = car_parameters_->tire_parameters->rl_camber;
    } else {
      tire_input.distance_to_CG = car_parameters_->tire_parameters->d_bright;
      tire_input.camber_angle = car_parameters_->tire_parameters->rr_camber;
    }

    calculate_slip_angle_rear_not_transient(tire_input);
  }

  calculate_slip_ratio_not_transient(tire_input);

  auto forces = this->tire_forces(tire_input);
  constexpr double kLinearSlipStiffness = 34.6;  // dFx/dkappa per unit Fz; tuned, not derived.
  constexpr double kBlendCenter = 5.0;           // [m/s]
  constexpr double kBlendWidth = 2.0;            // [m/s]

  double Fx_linear = kLinearSlipStiffness * tire_input.vertical_load * tire_input.slip_ratio;
  // double Fx_limit = car_parameters_->tire_parameters->PDX1 * tire_input.vertical_load;
  // Fx_linear = std::clamp(Fx_linear, -Fx_limit, Fx_limit);
  double speed_blend =
      0.5 * (std::tanh((std::abs(tire_input.vx) - kBlendCenter) / kBlendWidth) + 1.0);
  forces(0) = (1.0 - speed_blend) * Fx_linear + speed_blend * forces(0);
  return forces;
}


// Transient
void TireModel::calculate_slip_angle_front(TireInput& tire_input) {
  double L_alpha = car_parameters_->tire_parameters->slip_angle_relaxation_length;
  if (L_alpha <= 0.001) {
    calculate_slip_angle_front_not_transient(tire_input);
    return;
  }

  constexpr double longitudinal_epsilon = 0.5;

  const Eigen::Vector2d contact_patch =
      calculate_contact_patch_velocity(tire_input, car_parameters_->track_width);
  const double Vcx = contact_patch.x();
  const double Vcy = contact_patch.y();
  tire_input.vcx = Vcx;
  tire_input.vcy = Vcy;

  const double Vlong_reg = std::hypot(Vcx, longitudinal_epsilon);
  const double direction = Vcx / Vlong_reg;
  const double alpha_target = std::atan2(Vcy, Vlong_reg) * direction;
  const double V_mag = std::max(std::abs(Vcx), 0.5);  // Avoid division by zero

  const double relaxation = std::exp(-(V_mag / L_alpha) * tire_input.dt);
  tire_input.slip_angle =
      alpha_target + (tire_input.last_slip_angle[tire_input.tire] - alpha_target) * relaxation;
  tire_input.last_slip_angle[tire_input.tire] = tire_input.slip_angle;
}

void TireModel::calculate_slip_angle_rear(TireInput& tire_input) {
  double L_alpha = car_parameters_->tire_parameters->slip_angle_relaxation_length;
  if (L_alpha <= 0.001) {
    calculate_slip_angle_rear_not_transient(tire_input);
    return;
  }

  constexpr double longitudinal_epsilon = 0.5;

  const Eigen::Vector2d contact_patch =
      calculate_contact_patch_velocity(tire_input, car_parameters_->track_width);
  const double Vcx = contact_patch.x();
  const double Vcy = contact_patch.y();
  tire_input.vcx = Vcx;
  tire_input.vcy = Vcy;

  const double Vlong_reg = std::hypot(Vcx, longitudinal_epsilon);
  const double direction = Vcx / Vlong_reg;
  const double alpha_target = std::atan2(Vcy, Vlong_reg) * direction;
  double V_mag = std::max(std::abs(Vcx), 0.5);

  double relaxation = std::exp(-(V_mag / L_alpha) * tire_input.dt);
  tire_input.slip_angle =
      alpha_target + (tire_input.last_slip_angle[tire_input.tire] - alpha_target) * relaxation;
  tire_input.last_slip_angle[tire_input.tire] = tire_input.slip_angle;
}

void TireModel::calculate_slip_ratio(TireInput& tire_input) {
  double L = car_parameters_->tire_parameters->slip_ratio_relaxation_length;
  if (L <= 0.001) {
    calculate_slip_ratio_not_transient(tire_input);
    return;
  }

  constexpr double V_floor = 1.0;

  const Eigen::Vector2d contact_patch =
      calculate_contact_patch_velocity(tire_input, car_parameters_->track_width);
  const double Vcx = contact_patch.x();
  tire_input.vcx = Vcx;
  tire_input.vcy = contact_patch.y();
  double Vw = tire_input.wheel_angular_speed * car_parameters_->tire_parameters->effective_tire_r;

  if (std::abs(Vcx) < 0.01 && std::abs(Vw) < 0.01) {
    tire_input.slip_ratio = 0.0;
    tire_input.last_slip_ratio[tire_input.tire] = 0.0;
    return;
  }

  const double denominator = std::max({std::abs(Vw), std::abs(Vcx), V_floor});
  const double slip_target = std::clamp((Vw - Vcx) / denominator, -1.0, 1.0);
  double V_mag = std::max(std::abs(Vcx), 0.5);

  double relaxation = std::exp(-(V_mag / L) * tire_input.dt);
  tire_input.slip_ratio =
      slip_target + (tire_input.last_slip_ratio[tire_input.tire] - slip_target) * relaxation;
  tire_input.slip_ratio = std::clamp(tire_input.slip_ratio, -1.0, 1.0);

  tire_input.last_slip_ratio[tire_input.tire] = tire_input.slip_ratio;

}

Eigen::Vector4d TireModel::calculate_tire_forces(TireInput& tire_input) {
  if (tire_input.tire == FL || tire_input.tire == FR) {
    if (tire_input.tire == FL) {
      tire_input.distance_to_CG = car_parameters_->tire_parameters->d_fleft;
      tire_input.camber_angle = car_parameters_->tire_parameters->fl_camber;
    } else {
      tire_input.distance_to_CG = car_parameters_->tire_parameters->d_fright;
      tire_input.camber_angle = car_parameters_->tire_parameters->fr_camber;
    }
    calculate_slip_angle_front(tire_input);
  } else {
    if (tire_input.tire == RL) {
      tire_input.distance_to_CG = car_parameters_->tire_parameters->d_bleft;
      tire_input.camber_angle = car_parameters_->tire_parameters->rl_camber;
    } else {
      tire_input.distance_to_CG = car_parameters_->tire_parameters->d_bright;
      tire_input.camber_angle = car_parameters_->tire_parameters->rr_camber;
    }
    calculate_slip_angle_rear(tire_input);
  }

  calculate_slip_ratio(tire_input);

  // Return tire forces using the specific derived tire model
  return this->tire_forces(tire_input);
}