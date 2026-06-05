#include "motion_lib/tire_model/base_tire_model.hpp"

void TireModel::calculate_slip_angle_front(TireInput& tire_input) {
  const double V_eps = 0.5;

  // Sign used to apply the effect of yaw_rate
  double sign_y = (tire_input.tire == FL || tire_input.tire == RL) ? -1.0 : 1.0;
  double lf = tire_input.distance_to_CG;

  // Wheel velocities at vehicle frame
  double v_wheel_x =
      tire_input.vx + (sign_y * tire_input.yaw_rate * car_parameters_->track_width / 2.0);
  double v_wheel_y = tire_input.vy + (tire_input.yaw_rate * lf);

  // Projection to wheels frame
  double Vcx =
      v_wheel_x * cos(tire_input.steering_angle) + v_wheel_y * sin(tire_input.steering_angle);
  double Vcy =
      -v_wheel_x * sin(tire_input.steering_angle) + v_wheel_y * cos(tire_input.steering_angle);

  // Target slip angle
  double alpha_target = 0.0;
  if (std::sqrt(Vcx * Vcx + Vcy * Vcy) >= 0.05) {
    double Vlong_reg = std::sqrt(Vcx * Vcx + (V_eps * V_eps));
    double direction = (Vcx >= 0.0) ? 1.0 : -1.0;
    alpha_target = atan2(Vcy, Vlong_reg) * direction;
  }

  // Transient slip angle
  double L_alpha = car_parameters_->tire_parameters->relaxation_length;
  double V_mag = std::max(std::abs(Vcx), 0.5);  // Avoid division by zero

  // First-order lag system: d(alpha)/dt = (V / L) * (alpha_target - alpha_current)
  double alpha_derivative =
      (V_mag / L_alpha) * (alpha_target - tire_input.last_slip_angle[tire_input.tire]);

  // Integrate
  tire_input.slip_angle =
      tire_input.last_slip_angle[tire_input.tire] + (alpha_derivative * tire_input.dt);

  // Update state for next frame
  tire_input.last_slip_angle[tire_input.tire] = tire_input.slip_angle;
}

void TireModel::calculate_slip_angle_rear(TireInput& tire_input) {
  const double V_eps = 0.5;

  // Sign used to apply the effect of yaw_rate
  double sign_y = (tire_input.tire == FL || tire_input.tire == RL) ? -1.0 : 1.0;
  double lr = tire_input.distance_to_CG;

  // Wheel velocities at vehicle frame
  double v_wheel_x =
      tire_input.vx + (sign_y * tire_input.yaw_rate * car_parameters_->track_width / 2.0);
  double v_wheel_y = tire_input.vy - (tire_input.yaw_rate * lr);

  // Project to wheel frame
  double Vcx = v_wheel_x;
  double Vcy = v_wheel_y;

  // Target slip angle
  double alpha_target = 0.0;
  if (std::sqrt(Vcx * Vcx + Vcy * Vcy) >= 0.05) {
    double Vlong_reg = std::sqrt(Vcx * Vcx + (V_eps * V_eps));
    double direction = (Vcx >= 0.0) ? 1.0 : -1.0;
    alpha_target = atan2(Vcy, Vlong_reg) * direction;
  }

  // Transient slip angle
  double L_alpha = car_parameters_->tire_parameters->relaxation_length;
  double V_mag = std::max(std::abs(Vcx), 0.5);

  // First-order lag system: d(alpha)/dt = (V / L) * (alpha_target - alpha_current)
  double alpha_derivative =
      (V_mag / L_alpha) * (alpha_target - tire_input.last_slip_angle[tire_input.tire]);

  // Integrate
  tire_input.slip_angle =
      tire_input.last_slip_angle[tire_input.tire] + (alpha_derivative * tire_input.dt);

  // Update state for next frame
  tire_input.last_slip_angle[tire_input.tire] = tire_input.slip_angle;
}

void TireModel::calculate_slip_angle_front_not_transient(TireInput& tire_input) {
  const double V_eps = 0.5;
  const double half_track = car_parameters_->track_width / 2.0;

  // Wheel position relative to the CG (ISO: x forward, y left, yaw positive CCW).
  // Left wheels sit at +track/2, right wheels at -track/2. Use the longitudinal
  // CG distance lf, not the diagonal d_f* distance.
  double y_sign = (tire_input.tire == FL) ? 1.0 : -1.0;
  double lf = car_parameters_->wheelbase - car_parameters_->cg_2_rear_axis;

  // Hub velocity in the vehicle frame: v = v_cg + omega x r.
  // The lateral offset (track) enters the longitudinal velocity; lf enters the lateral.
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
  // See calculate_slip_angle_front_not_transient for the regularization rationale.
  const double V_eps = 0.5;
  const double half_track = car_parameters_->track_width / 2.0;

  // Left wheels sit at +track/2, right wheels at -track/2. Use the longitudinal
  // CG distance lr, not the diagonal d_b* distance.
  double y_sign = (tire_input.tire == RL) ? 1.0 : -1.0;
  double lr = car_parameters_->cg_2_rear_axis;

  // Hub velocity in the vehicle frame (rear wheels are unsteered).
  // The lateral offset (track) enters the longitudinal velocity; lr enters the lateral.
  double vx_hub = tire_input.vx - (tire_input.yaw_rate * y_sign * half_track);
  double vy_hub = tire_input.vy - (tire_input.yaw_rate * lr);

  // Smooth denominator + smooth sign(vx); slip_angle -> 0 smoothly at standstill.
  double Vlong_reg = std::sqrt(vx_hub * vx_hub + V_eps * V_eps);
  double direction = vx_hub / Vlong_reg;
  tire_input.slip_angle = atan2(vy_hub, Vlong_reg) * direction;

  tire_input.last_slip_angle[tire_input.tire] = tire_input.slip_angle;
}

void TireModel::calculate_slip_ratio(TireInput& tire_input) {
  double sign_y = (tire_input.tire == FL || tire_input.tire == RL) ? -1.0 : 1.0;

  // Wheels velocities in the vehicle frame
  double v_wheel_x =
      tire_input.vx + (sign_y * tire_input.yaw_rate * car_parameters_->track_width / 2.0);
  double v_wheel_y = tire_input.vy;
  if (tire_input.tire == FL || tire_input.tire == FR) {
    v_wheel_y += (tire_input.yaw_rate * tire_input.distance_to_CG);
  } else {
    v_wheel_y -= (tire_input.yaw_rate * tire_input.distance_to_CG);
  }

  // Project to the wheel frame
  double Vcx =
      v_wheel_x * cos(tire_input.steering_angle) + v_wheel_y * sin(tire_input.steering_angle);
  double Vw = tire_input.wheel_angular_speed * car_parameters_->tire_parameters->effective_tire_r;

  // Low speed stabilization
  if (std::abs(Vcx) < 0.01 && std::abs(Vw) < 0.01) {
    tire_input.slip_ratio = 0.0;
    tire_input.last_slip_ratio[tire_input.tire] = 0.0;
    return;
  }

  // Target slip ratio
  double stabilizer_epsilon = 0.1;
  double denominator = std::sqrt(Vcx * Vcx + stabilizer_epsilon * stabilizer_epsilon);
  double slip_target = (Vw - Vcx) / denominator;
  slip_target = std::clamp(slip_target, -1.0, 1.0);

  // Transient slip ratio
  double L = car_parameters_->tire_parameters->relaxation_length;
  double V_mag = std::max(std::abs(Vcx), 0.5);

  // First-order lag system: d(slip_ratio)/dt = (V / L) * (slip_target - slip_current)
  double slip_derivative =
      (V_mag / L) * (slip_target - tire_input.last_slip_ratio[tire_input.tire]);

  // Integrate
  tire_input.slip_ratio =
      tire_input.last_slip_ratio[tire_input.tire] + (slip_derivative * tire_input.dt);
      tire_input.slip_ratio = std::clamp(tire_input.slip_ratio, -1.0, 1.0);

  // Update state for next frame
  tire_input.last_slip_ratio[tire_input.tire] = tire_input.slip_ratio;
  
}

void TireModel::calculate_slip_ratio_not_transient(TireInput& tire_input) {
  const double half_track = car_parameters_->track_width / 2.0;
  
  // 1. Identify distances from CG. Both should be stored or calculated as positive numbers.
  bool is_rear = (tire_input.tire == RL || tire_input.tire == RR);
  double l_axle = is_rear ? car_parameters_->cg_2_rear_axis
                          : (car_parameters_->wheelbase - car_parameters_->cg_2_rear_axis);

  // 2. Wheel positions relative to CG in ISO coordinates (Left is positive Y, Right is negative Y)
  double y_sign = (tire_input.tire == FL || tire_input.tire == RL) ? 1.0 : -1.0;
  // Front is positive X, Rear is negative X
  double x_sign = is_rear ? -1.0 : 1.0;

  // 3. Rigid-body translation (V = V_cg + w x r)
  double vx_hub = tire_input.vx - (tire_input.yaw_rate * y_sign * half_track);
  double vy_hub = tire_input.vy + (tire_input.yaw_rate * x_sign * l_axle);

  // 4. Longitudinal velocity at the wheel patch
  double Vcx = vx_hub * std::cos(tire_input.steering_angle) + vy_hub * std::sin(tire_input.steering_angle);
  double Vw = tire_input.wheel_angular_speed * car_parameters_->tire_parameters->effective_tire_r;

  // 5. Normalize by the larger of the two surface speeds (with a small floor) rather
  //    than by Vcx alone. A Vcx-normalized ratio is singular at low speed: its slope
  //    d(kappa)/d(omega) = r / Vcx blows up as Vcx -> 0, so the force chatters and kappa
  //    slams to +/-1. This form is bounded in [-1, 1] by construction, its slope
  //    ~ Vcx / Vw^2 stays small at low speed (no chatter), it matches the standard slip
  //    ratio at speed, and below V_floor it degrades gracefully into a creep model
  //    (kappa proportional to slip velocity).
  const double V_floor = 1.0;  // [m/s]
  double denom = std::max({std::abs(Vw), std::abs(Vcx), V_floor});
  double slip_target = std::clamp((Vw - Vcx) / denom, -1.0, 1.0);

  tire_input.slip_ratio = slip_target;
  tire_input.last_slip_ratio[tire_input.tire] = slip_target;
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

  // Return tire forces using the specific tire model
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