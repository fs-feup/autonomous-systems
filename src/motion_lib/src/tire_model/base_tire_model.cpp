#include "motion_lib/tire_model/base_tire_model.hpp"

void TireModel::calculateSlipAngleFront(TireInput& tire_input) {
  const double V_eps = 1;  // Regularization constant to prevent incorrect low speed behavior can
                           // be lower if needed
  double sign = (tire_input.tire == FL) ? -1.0 : 1.0;  // Sign used to apply the effect of yaw_rate

  // Normalize velocity to wheels coordinate system
  double Vcx = tire_input.vx * cos(tire_input.steering_angle) +
               tire_input.vy * sin(tire_input.steering_angle);
  double Vcy = -tire_input.vx * sin(tire_input.steering_angle) +
               tire_input.vy * cos(tire_input.steering_angle);

  tire_input.slip_angle =
      (atan((Vcy + (tire_input.yaw_rate * tire_input.distance_to_CG) +
             (sign * tire_input.yaw_rate * car_parameters_->track_width / 2.0)) /
            sqrt(Vcx * Vcx + V_eps * V_eps)));
}

void TireModel::calculateSlipAngleRear(TireInput& tire_input) {
  const double V_eps = 1;
  double sign = (tire_input.tire == RL) ? -1.0 : 1.0;

  // Lateral velocity at the wheel contact patch
  double Vcy_contact = tire_input.vy - (tire_input.yaw_rate * tire_input.distance_to_CG) +
                       (sign * tire_input.yaw_rate * car_parameters_->track_width / 2.0);

  tire_input.slip_angle = atan(Vcy_contact / sqrt(tire_input.vx * tire_input.vx + V_eps * V_eps));
}

void TireModel::calculateSlipRatio(TireInput& tire_input) {
  const double V_eps = 1;  // Regularization constant to prevent incorrect low speed behavior can
                           // be lower if needed
  // Normalize velocity to wheels coordinate system
  double Vcx = tire_input.vx * cos(tire_input.steering_angle) +
               tire_input.vy * sin(tire_input.steering_angle);
  double slip_ratio =
      (tire_input.wheel_angular_speed * car_parameters_->tire_parameters->effective_tire_r - Vcx) /
      sqrt(Vcx * Vcx + V_eps * V_eps);
  slip_ratio = std::clamp(slip_ratio, -1.0, 1.0);  // hard physical limit
  tire_input.slip_ratio = slip_ratio;
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