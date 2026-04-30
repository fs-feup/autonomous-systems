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

  // Update state for next frame
  tire_input.last_slip_ratio[tire_input.tire] = tire_input.slip_ratio;
}

void TireModel::calculate_slip_ratio_not_transient(TireInput& tire_input) {
  // 1. Calculate the Wheel Longitudinal Velocity with yaw
  // Sign: Left wheels move slower in a positive yaw (turning left), Right move faster.
  double sign = (tire_input.tire == FL || tire_input.tire == RL) ? -1.0 : 1.0;

  // Longitudinal velocity at the wheel patch
  double Vcx_center = tire_input.vx * cos(tire_input.steering_angle) +
                      tire_input.vy * sin(tire_input.steering_angle);

  // Add the yaw component (tangential velocity)
  double Vcx = Vcx_center + (sign * tire_input.yaw_rate * car_parameters_->track_width / 2.0);

  double Vw = tire_input.wheel_angular_speed * car_parameters_->tire_parameters->effective_tire_r;

  if (Vcx < 0.01 || Vw < 0.01) {
    tire_input.slip_ratio = 0.0;
    return;
  }

  // Calculate the "Target" (Steady-State) Slip
  double slip_target = (Vw - Vcx) / std::abs(Vcx);
  slip_target = std::clamp(slip_target, -1.0, 1.0);

  tire_input.slip_ratio = slip_target;
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

Eigen::Vector3d TireModel::calculate_tire_forces_not_transient(TireInput& tire_input) {
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

  // Return tire forces using the specific tire model
  return this->tire_forces(tire_input);
}