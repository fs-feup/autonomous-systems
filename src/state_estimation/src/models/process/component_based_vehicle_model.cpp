#include "models/process/component_based_vehicle_model.hpp"

ComponentBasedVehicleModel::ComponentBasedVehicleModel(
    const std::shared_ptr<SEParameters>& parameters)
    : ProcessModel(parameters) {
  this->differential_model_ =
      differential_models_map.at(parameters->differential_model_name_)(parameters->car_parameters_);
  this->load_transfer_model_ = load_transfer_models_map.at(parameters->load_transfer_model_name_)(
      parameters->car_parameters_);
  this->aero_model_ = aero_models_map.at(parameters->aero_model_name_)(parameters->car_parameters_);
  this->steering_model_ =
      steering_models_map.at(parameters->steering_model_name_)(parameters->car_parameters_);
  this->steering_motor_model_ = steering_motor_models_map.at(
      parameters->steering_motor_model_name_)(parameters->car_parameters_);
  this->tire_model_ = tire_models_map.at(parameters->tire_model_name_)(parameters->car_parameters_);
}

void ComponentBasedVehicleModel::predict(Eigen::Ref<State> state,
                                         common_lib::structures::ControlCommand control_command,
                                         double dt) {
  // Scale control command to torque (02 version, assuming single motor RWD)
  double throttle_input =
      control_command.throttle_rl * parameters_->car_parameters_->motor_parameters->max_peak_torque;

  // Calculate torque distribution using the differential model
  common_lib::structures::Wheels wheel_speeds;
  wheel_speeds.front_left = state(FL_WHEEL_SPEED);
  wheel_speeds.front_right = state(FR_WHEEL_SPEED);
  wheel_speeds.rear_left = state(RL_WHEEL_SPEED);
  wheel_speeds.rear_right = state(RR_WHEEL_SPEED);

  common_lib::structures::Wheels torques_struct =
      differential_model_->calculateTorqueDistribution(throttle_input, wheel_speeds);
  Eigen::Vector4d torques(torques_struct.front_left, torques_struct.front_right,
                          torques_struct.rear_left, torques_struct.rear_right);

  // Calculate individual wheel yaw using the steering model [front left, front right, rear left,
  // rear right]
  Eigen::Vector4d wheel_angles = this->steering_model_->calculate_steering_angles(state(ST_ANGLE));

  // Calculate load in each tire using the load transfer model
  LoadTransferInput load_transfer_input;
  load_transfer_input.longitudinal_acceleration = state(AX);
  load_transfer_input.lateral_acceleration = state(AY);
  common_lib::structures::Wheels load_distribution =
      load_transfer_model_->compute_loads(load_transfer_input);

  // Calculate aerodynamic forces using the aero model
  Eigen::Vector3d aero_forces = aero_model_->aero_forces(state.segment<3>(VX));

  // Compute load + aero downforce
  double front_aero_balance =
      this->parameters_->car_parameters_->aero_parameters->aero_balance_front;
  Eigen::Vector4d total_vertical_loads(
      load_distribution.front_left + aero_forces(2) * front_aero_balance * 0.5,
      load_distribution.front_right + aero_forces(2) * front_aero_balance * 0.5,
      load_distribution.rear_left + aero_forces(2) * (1 - front_aero_balance) * 0.5,
      load_distribution.rear_right + aero_forces(2) * (1 - front_aero_balance) * 0.5);

  // TIRE MODEL
  TireInput tire_input;
  Eigen::VectorXd tire_forces = Eigen::VectorXd(12);  // 4 tires * 3 forces each
  tire_input.vx = state(VX);
  tire_input.vy = state(VY);
  tire_input.yaw_rate = state(YAW_RATE);
  for (Tire tire : {FL, FR, RL, RR}) {
    tire_input.tire = tire;
    tire_input.steering_angle = wheel_angles(tire);
    tire_input.wheel_angular_speed = state(FL_WHEEL_SPEED + tire);
    tire_input.vertical_load = total_vertical_loads(tire);
    tire_forces.segment<3>(tire * 3) = tire_model_->calculateTireForces(tire_input);  //[Fx, Fy, Fz]
  }

  // Calculate steering rate using the steering motor model
  double steering_rate =
      steering_motor_model_->compute_steering_rate(state(ST_ANGLE), control_command.steering_angle);

  // Update state using the calculated values

  // Update steering angle
  state(ST_ANGLE) += steering_rate * dt;

  double total_fx = aero_forces(0);
  double total_fy = aero_forces(1);
  double total_torque = 0.0;
  double lr = parameters_->car_parameters_->cg_2_rear_axis;  // distance from CG to rear axle
  double lf = parameters_->car_parameters_->wheelbase - lr;  // distance from CG to front axle
  double half_width = parameters_->car_parameters_->track_width *
                      0.5;  // half of the track width for moment arm calculations
  double wheel_radius = parameters_->car_parameters_->tire_parameters->effective_tire_r;
  double inertia = parameters_->car_parameters_->tire_parameters->wheel_inertia;

  for (Tire tire : {FL, FR, RL, RR}) {
    // Update wheel speeds using the calculated torques and tire forces
    state(FL_WHEEL_SPEED + tire) +=
        ((torques(tire) - tire_forces(tire * 3) * wheel_radius) / inertia) *
        dt;  // No braking torque

    // Current tire forces in tire-local frame
    double fx_tire = tire_forces(tire * 3);
    double fy_tire = tire_forces(tire * 3 + 1);
    double mz_tire = tire_forces(tire * 3 + 2);  // Self-aligning torque

    // Transform to vehicle frame
    double cos_delta = cos(wheel_angles(tire));
    double sin_delta = sin(wheel_angles(tire));

    double fx_veh = fx_tire * cos_delta - fy_tire * sin_delta;
    double fy_veh = fx_tire * sin_delta + fy_tire * cos_delta;

    total_fx += fx_veh;
    total_fy += fy_veh;

    // Calculate Moment arms
    double arm_x = (tire == FL || tire == FR) ? lf : -lr;
    double arm_y = (tire == FL || tire == RL) ? -half_width : half_width;

    // Sum moments
    total_torque += (arm_x * fy_veh) - (arm_y * fx_veh) + mz_tire;
  }

  // Update Accelerations
  double total_ax =
      total_fx / parameters_->car_parameters_->total_mass + state(VY) * state(YAW_RATE);
  double total_ay =
      total_fy / parameters_->car_parameters_->total_mass - state(VX) * state(YAW_RATE);

  // Integration via trapezoidal rule
  state(VX) += (0.5 * (total_ax + state(AX))) * dt;
  state(VY) += (0.5 * (total_ay + state(AY))) * dt;
  state(AX) = total_ax;
  state(AY) = total_ay;

  // Update Yaw Rate
  state(YAW_RATE) += (total_torque / parameters_->car_parameters_->Izz) * dt;
};