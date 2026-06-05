#include "motion_lib/tire_model/base_tire_model.hpp"

#include <algorithm>
#include <cmath>

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

void TireModel::calculate_slip_angle_front(TireInput& tire_input) {
  constexpr double min_contact_patch_speed = 0.05;
  constexpr double longitudinal_epsilon = 0.1;

  const Eigen::Vector2d contact_patch =
      calculate_contact_patch_velocity(tire_input, car_parameters_->track_width);
  const double Vcx = contact_patch.x();
  const double Vcy = contact_patch.y();
  tire_input.contact_patch_longitudinal_velocity = Vcx;
  tire_input.contact_patch_lateral_velocity = Vcy;

  double alpha_target = 0.0;
  if (std::hypot(Vcx, Vcy) >= min_contact_patch_speed) {
    double direction = (Vcx >= 0.0) ? 1.0 : -1.0;
    alpha_target = std::atan2(Vcy, std::abs(Vcx) + longitudinal_epsilon) * direction;
  }

  double L_alpha = car_parameters_->tire_parameters->slip_angle_relaxation_length;
  double V_mag = std::max(std::abs(Vcx), 0.5);  // Avoid division by zero

  if (L_alpha > 0.0) {
    double relaxation = std::exp(-(V_mag / L_alpha) * tire_input.dt);
    tire_input.slip_angle =
        alpha_target + (tire_input.last_slip_angle[tire_input.tire] - alpha_target) * relaxation;
  } else {
    tire_input.slip_angle = alpha_target;
  }

  // Update state for next frame
  tire_input.last_slip_angle[tire_input.tire] = tire_input.slip_angle;
}

void TireModel::calculate_slip_angle_rear(TireInput& tire_input) {
  constexpr double min_contact_patch_speed = 0.05;
  constexpr double longitudinal_epsilon = 0.1;

  const Eigen::Vector2d contact_patch =
      calculate_contact_patch_velocity(tire_input, car_parameters_->track_width);
  const double Vcx = contact_patch.x();
  const double Vcy = contact_patch.y();
  tire_input.contact_patch_longitudinal_velocity = Vcx;
  tire_input.contact_patch_lateral_velocity = Vcy;

  double alpha_target = 0.0;
  if (std::hypot(Vcx, Vcy) >= min_contact_patch_speed) {
    double direction = (Vcx >= 0.0) ? 1.0 : -1.0;
    alpha_target = std::atan2(Vcy, std::abs(Vcx) + longitudinal_epsilon) * direction;
  }

  double L_alpha = car_parameters_->tire_parameters->slip_angle_relaxation_length;
  double V_mag = std::max(std::abs(Vcx), 0.5);

  if (L_alpha > 0.0) {
    double relaxation = std::exp(-(V_mag / L_alpha) * tire_input.dt);
    tire_input.slip_angle =
        alpha_target + (tire_input.last_slip_angle[tire_input.tire] - alpha_target) * relaxation;
  } else {
    tire_input.slip_angle = alpha_target;
  }

  // Update state for next frame
  tire_input.last_slip_angle[tire_input.tire] = tire_input.slip_angle;
}

void TireModel::calculate_slip_ratio(TireInput& tire_input) {
  constexpr double min_slip_denominator = 0.1;

  const Eigen::Vector2d contact_patch =
      calculate_contact_patch_velocity(tire_input, car_parameters_->track_width);
  const double Vcx = contact_patch.x();
  tire_input.contact_patch_longitudinal_velocity = Vcx;
  tire_input.contact_patch_lateral_velocity = contact_patch.y();
  double Vw = tire_input.wheel_angular_speed * car_parameters_->tire_parameters->effective_tire_r;

  if (std::abs(Vcx) < 0.01 && std::abs(Vw) < 0.01) {
    tire_input.slip_ratio = 0.0;
    tire_input.last_slip_ratio[tire_input.tire] = 0.0;
    return;
  }

  double denominator = std::max(std::abs(Vcx), min_slip_denominator);
  double slip_target = (Vw - Vcx) / denominator;

  double L = car_parameters_->tire_parameters->slip_ratio_relaxation_length;
  double V_mag = std::max(std::abs(Vcx), 0.5);

  if (L > 0.0) {
    double relaxation = std::exp(-(V_mag / L) * tire_input.dt);
    tire_input.slip_ratio =
        slip_target + (tire_input.last_slip_ratio[tire_input.tire] - slip_target) * relaxation;
  } else {
    tire_input.slip_ratio = slip_target;
  }

  // Update state for next frame
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
