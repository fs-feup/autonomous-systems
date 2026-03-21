#include "motion_lib/tire_model/base_tire_model.hpp"

void TireModel::calculateSlipAngleFront(TireInput& tire_input) {
  const double V_eps = 0.5;  // Regularization constant to prevent incorrect low speed behavior can
                             // be lower if needed
  double sign = (tire_input.tire == FL) ? -1.0 : 1.0;  // Sign used to apply the effect of yaw_rate

  // Normalize velocity to wheels coordinate system
  double Vcx = tire_input.vx * cos(tire_input.steering_angle) +
               tire_input.vy * sin(tire_input.steering_angle);
  double Vcy = -tire_input.vx * sin(tire_input.steering_angle) +
               tire_input.vy * cos(tire_input.steering_angle);
  if (std::sqrt(Vcx * Vcx + Vcy * Vcy) < 0.05) {
    tire_input.slip_angle = 0.0;
  } else {
    tire_input.slip_angle =
        atan((Vcy + (tire_input.yaw_rate * tire_input.distance_to_CG) +
              (sign * tire_input.yaw_rate * car_parameters_->track_width / 2.0)) /
             sqrt(Vcx * Vcx + (V_eps * V_eps)));
  }
}

void TireModel::calculateSlipAngleRear(TireInput& tire_input) {
  const double V_eps = 0.5;
  double sign = (tire_input.tire == RL) ? -1.0 : 1.0;

  // Lateral velocity at the wheel contact patch
  double Vcy_contact = tire_input.vy - (tire_input.yaw_rate * tire_input.distance_to_CG) +
                       (sign * tire_input.yaw_rate * car_parameters_->track_width / 2.0);
  if (std::sqrt(tire_input.vx * tire_input.vx + Vcy_contact * Vcy_contact) < 0.05) {
    tire_input.slip_angle = 0.0;
  } else {
    tire_input.slip_angle =
        atan(Vcy_contact / sqrt(tire_input.vx * tire_input.vx + (V_eps * V_eps)));
  }
}

void TireModel::calculateSlipRatio(TireInput& tire_input) {
  // 1. Calculate the Wheel Longitudinal Velocity with yaw
  // Sign: Left wheels move slower in a positive yaw (turning left), Right move faster.
  double sign = (tire_input.tire == FL || tire_input.tire == RL) ? -1.0 : 1.0;

  // Longitudinal velocity at the wheel patch
  double Vcx_center = tire_input.vx * cos(tire_input.steering_angle) +
                      tire_input.vy * sin(tire_input.steering_angle);

  // Add the yaw component (tangential velocity)
  double Vcx = Vcx_center + (sign * tire_input.yaw_rate * car_parameters_->track_width / 2.0);

  double Vw = tire_input.wheel_angular_speed * car_parameters_->tire_parameters->effective_tire_r;

  // 2. Calculate the "Target" (Steady-State) Slip
  double denominator = std::sqrt(Vcx * Vcx + 0.1);
  double slip_target = (Vw - Vcx) / denominator;
  slip_target = std::clamp(slip_target, -1.0, 1.0);

  // 3. Apply Relaxation Length (Transient Logic)
  // L is the distance the tire must roll to build up full force
  double L = car_parameters_->tire_parameters->relaxation_length;

  // If the car is stopped, we use a minimum speed to allow slip to decay.
  double V_mag = std::max(std::abs(Vcx), 0.5);

  // First-order lag: dS/dt = (V / L) * (target_S - current_S)
  double slip_derivative =
      (V_mag / L) * (slip_target - tire_input.last_slip_ratio[tire_input.tire]);

  // Integrate
  tire_input.slip_ratio =
      tire_input.last_slip_ratio[tire_input.tire] + (slip_derivative * tire_input.dt);

  // 4. Update state for next frame
  tire_input.last_slip_ratio[tire_input.tire] = tire_input.slip_ratio;
}

Eigen::Vector3d TireModel::calculateTireForces(TireInput& tire_input) {
  if (tire_input.tire == FL || tire_input.tire == FR) {
    if (tire_input.tire == FL) {
      tire_input.distance_to_CG = car_parameters_->tire_parameters->d_fleft;
      tire_input.camber_angle = car_parameters_->tire_parameters->fl_camber;
    } else {
      tire_input.distance_to_CG = car_parameters_->tire_parameters->d_fright;
      tire_input.camber_angle = car_parameters_->tire_parameters->fr_camber;
    }

    calculateSlipAngleFront(tire_input);
  } else {
    if (tire_input.tire == RL) {
      tire_input.distance_to_CG = car_parameters_->tire_parameters->d_bleft;
      tire_input.camber_angle = car_parameters_->tire_parameters->rl_camber;
    } else {
      tire_input.distance_to_CG = car_parameters_->tire_parameters->d_bright;
      tire_input.camber_angle = car_parameters_->tire_parameters->rr_camber;
    }

    calculateSlipAngleRear(tire_input);
  }

  calculateSlipRatio(tire_input);
  // Return tire forces using the specific tire model
  return this->tire_forces(tire_input);
}