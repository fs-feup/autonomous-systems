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
  constexpr double longitudinal_epsilon = 0.5;

  const Eigen::Vector2d contact_patch =
      calculate_contact_patch_velocity(tire_input, car_parameters_->track_width);
  const double Vcx = contact_patch.x();
  const double Vcy = contact_patch.y();
  tire_input.vcx = Vcx;
  tire_input.vcy = Vcy;

  const double Vlong_reg = std::hypot(Vcx, longitudinal_epsilon);
  const double direction = Vcx / Vlong_reg;
  tire_input.slip_angle = std::atan2(Vcy, Vlong_reg) * direction;
  tire_input.last_slip_angle[tire_input.tire] = tire_input.slip_angle;
}

void TireModel::calculate_slip_angle_rear_not_transient(TireInput& tire_input) {
  constexpr double longitudinal_epsilon = 0.5;

  const Eigen::Vector2d contact_patch =
      calculate_contact_patch_velocity(tire_input, car_parameters_->track_width);
  const double Vcx = contact_patch.x();
  const double Vcy = contact_patch.y();
  tire_input.vcx = Vcx;
  tire_input.vcy = Vcy;

  const double Vlong_reg = std::hypot(Vcx, longitudinal_epsilon);
  const double direction = Vcx / Vlong_reg;
  tire_input.slip_angle = std::atan2(Vcy, Vlong_reg) * direction;
  tire_input.last_slip_angle[tire_input.tire] = tire_input.slip_angle;
}

void TireModel::calculate_slip_ratio_not_transient(TireInput& tire_input) {
  constexpr double V_floor = 1.0;

  const Eigen::Vector2d contact_patch =
      calculate_contact_patch_velocity(tire_input, car_parameters_->track_width);
  const double Vcx = contact_patch.x();
  const double Vw =
      tire_input.wheel_angular_speed * car_parameters_->tire_parameters->effective_tire_r;
  tire_input.vcx = Vcx;
  tire_input.vcy = contact_patch.y();

  if (std::abs(Vcx) < 0.01 && std::abs(Vw) < 0.01) {
    tire_input.slip_ratio = 0.0;
    tire_input.last_slip_ratio[tire_input.tire] = 0.0;
    return;
  }

  const double denominator = std::max({std::abs(Vw), std::abs(Vcx), V_floor});
  tire_input.slip_ratio = std::clamp((Vw - Vcx) / denominator, -1.0, 1.0);
  tire_input.last_slip_ratio[tire_input.tire] = tire_input.slip_ratio;
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

  return this->tire_forces(tire_input);
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
