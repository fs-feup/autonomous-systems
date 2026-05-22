#include "models/process/rk4_model.hpp"

#include <algorithm>

RK4VehicleModel::RK4VehicleModel(const std::shared_ptr<SEParameters>& parameters)
    : ProcessModel(parameters) {
  this->transmission_model_ =
      transmission_models_map.at(parameters->transmission_model_name_)(parameters->car_parameters_);
  this->load_transfer_model_ = load_transfer_models_map.at(parameters->load_transfer_model_name_)(
      parameters->car_parameters_);
  this->aero_model_ = aero_models_map.at(parameters->aero_model_name_)(parameters->car_parameters_);
  this->steering_model_ =
      steering_models_map.at(parameters->steering_model_name_)(parameters->car_parameters_);
  this->steering_motor_model_ = steering_motor_models_map.at(
      parameters->steering_motor_model_name_)(parameters->car_parameters_);
  this->tire_model_ = tire_models_map.at(parameters->tire_model_name_)(parameters->car_parameters_);

  // Cache frequently accessed parameters
  lr_ = parameters->car_parameters_->cg_2_rear_axis;
  lf_ = parameters->car_parameters_->wheelbase - lr_;
  half_width_ = parameters->car_parameters_->track_width * 0.5;
  wheel_radius_ = parameters->car_parameters_->tire_parameters->effective_tire_r;
  inertia_ = parameters->car_parameters_->tire_parameters->wheel_inertia;
  total_mass_ = parameters->car_parameters_->total_mass;
  Izz_ = parameters->car_parameters_->Izz;
  max_peak_torque_ = parameters->car_parameters_->motor_parameters->max_peak_torque;
}

void RK4VehicleModel::predict(Eigen::Ref<State> state,
                              common_lib::structures::ControlCommand control_command, double dt) {
  Eigen::Matrix<double, StateSize, 1> f1 = get_state_derivative(state, control_command);
  State s2 = state + (0.5 * dt) * f1;
  Eigen::Matrix<double, StateSize, 1> f2 = get_state_derivative(s2, control_command);
  State s3 = state + (0.5 * dt) * f2;
  Eigen::Matrix<double, StateSize, 1> f3 = get_state_derivative(s3, control_command);
  State s4 = state + dt * f3;
  Eigen::Matrix<double, StateSize, 1> f4 = get_state_derivative(s4, control_command);

  // Combine to get new state
  state += (dt / 6.0) * (f1 + 2.0 * f2 + 2.0 * f3 + f4);
}

void RK4VehicleModel::compute_forces_and_moments(
    const State& state, common_lib::structures::ControlCommand control_command, double& total_fx,
    double& total_fy, double& total_torque) {
  if (abs(state(VX)) < 1e-2 && abs(control_command.throttle_rl) < 1e-6) {
    state_derivative_.setZero();
    return;
  }

  // Scale control command to torque
  double throttle_input = control_command.throttle_rl * max_peak_torque_;

  // Calculate torque distribution using transmission model (direct state access)
  common_lib::structures::Wheels wheel_speeds;
  wheel_speeds.front_left = state(FL_WHEEL_SPEED);
  wheel_speeds.front_right = state(FR_WHEEL_SPEED);
  wheel_speeds.rear_left = state(RL_WHEEL_SPEED);
  wheel_speeds.rear_right = state(RR_WHEEL_SPEED);

  common_lib::structures::Wheels torques_struct =
      transmission_model_->calculate_wheel_torques(throttle_input, wheel_speeds);
  torques_cache_(0) = torques_struct.front_left;
  torques_cache_(1) = torques_struct.front_right;
  torques_cache_(2) = torques_struct.rear_left;
  torques_cache_(3) = torques_struct.rear_right;

  // Calculate individual wheel yaw using steering model
  wheel_angles_cache_ = this->steering_model_->calculate_steering_angles(state(ST_ANGLE));

  // Calculate aerodynamic forces
  aero_forces_cache_ = aero_model_->aero_forces(state.segment<3>(VX));

  // Calculate load in each tire using load transfer model
  LoadTransferInput load_transfer_input;
  load_transfer_input.longitudinal_acceleration = state(AX);
  load_transfer_input.lateral_acceleration = state(AY);
  load_transfer_input.downforce = aero_forces_cache_(2);
  common_lib::structures::Wheels load_distribution =
      load_transfer_model_->compute_loads(load_transfer_input);

  total_vertical_loads_cache_(0) = load_distribution.front_left;
  total_vertical_loads_cache_(1) = load_distribution.front_right;
  total_vertical_loads_cache_(2) = load_distribution.rear_left;
  total_vertical_loads_cache_(3) = load_distribution.rear_right;

  // TIRE MODEL - compute forces for all tires
  TireInput tire_input;
  tire_input.vx = state(VX);
  tire_input.vy = state(VY);
  tire_input.yaw_rate = state(YAW_RATE);

  for (Tire tire : {FL, FR, RL, RR}) {
    tire_input.tire = tire;
    tire_input.steering_angle = wheel_angles_cache_(tire);
    tire_input.wheel_angular_speed = state(FL_WHEEL_SPEED + tire);
    tire_input.vertical_load = total_vertical_loads_cache_(tire);
    tire_forces_cache_.segment<4>(tire * 4) =
        tire_model_->calculate_tire_forces_not_transient(tire_input);  //[Fx, Fy, My, Mz]
  }

  // Calculate steering rate
  state_derivative_(ST_ANGLE) =
      steering_motor_model_->compute_steering_rate(state(ST_ANGLE), control_command.steering_angle);

  total_fx = aero_forces_cache_(0);
  total_fy = aero_forces_cache_(1);
  total_torque = 0.0;

  // Accumulate tire forces and moments
  for (Tire tire : {FL, FR, RL, RR}) {
    double fx_tire = tire_forces_cache_(tire * 4);
    double fy_tire = tire_forces_cache_(tire * 4 + 1);

    double cos_delta = cos(wheel_angles_cache_(tire));
    double sin_delta = sin(wheel_angles_cache_(tire));

    double fx_veh = fx_tire * cos_delta - fy_tire * sin_delta;
    double fy_veh = fx_tire * sin_delta + fy_tire * cos_delta;

    total_fx += fx_veh;
    total_fy += fy_veh;

    // Calculate moment arms (front: +lf, rear: -lr)
    double arm_x = (tire == FL || tire == FR) ? lf_ : -lr_;
    double arm_y = (tire == FL || tire == RL) ? half_width_ : -half_width_;

    total_torque += (arm_x * fy_veh) - (arm_y * fx_veh);
  }
}

Eigen::Matrix<double, StateSize, 1> RK4VehicleModel::get_state_derivative(
    Eigen::Ref<State> state, common_lib::structures::ControlCommand control_command) {
  double total_fx, total_fy, total_torque;
  compute_forces_and_moments(state, control_command, total_fx, total_fy, total_torque);

  // Compute accelerations
  state_derivative_(VX) = total_fx / total_mass_ + state(VY) * state(YAW_RATE);
  state_derivative_(VY) = total_fy / total_mass_ - state(VX) * state(YAW_RATE);

  // Acceleration derivatives
  state_derivative_(AX) = state_derivative_(VX) - state(AX);
  state_derivative_(AY) = state_derivative_(VY) - state(AY);

  // Yaw rate derivative
  state_derivative_(YAW_RATE) = total_torque / Izz_;

  // Wheel speed derivatives
  for (Tire tire : {FL, FR, RL, RR}) {
    state_derivative_(FL_WHEEL_SPEED + tire) =
        (torques_cache_(tire) - tire_forces_cache_(tire * 4) * wheel_radius_) / inertia_;
  }

  return state_derivative_;
}

VehicleState RK4VehicleModel::get_process_model_data(
    const State& state, const common_lib::structures::ControlCommand& control_command) {
  VehicleState vehicle_state;

  double total_fx, total_fy, total_torque;
  compute_forces_and_moments(state, control_command, total_fx, total_fy, total_torque);

  // Extract tire forces for output
  double total_moment_x = 0.0;
  double total_moment_y = 0.0;
  double total_self_aligning_moment = 0.0;

  for (Tire tire : {FL, FR, RL, RR}) {
    double fx_tire = tire_forces_cache_(tire * 4);
    double fy_tire = tire_forces_cache_(tire * 4 + 1);
    double mz_tire = tire_forces_cache_(tire * 4 + 3);

    double cos_delta = cos(wheel_angles_cache_(tire));
    double sin_delta = sin(wheel_angles_cache_(tire));

    double fx_veh = fx_tire * cos_delta - fy_tire * sin_delta;
    double fy_veh = fx_tire * sin_delta + fy_tire * cos_delta;

    double arm_x = (tire == FL || tire == FR) ? lf_ : -lr_;
    double arm_y = (tire == FL || tire == RL) ? half_width_ : -half_width_;

    total_moment_x += (arm_y * fy_veh) + (arm_x * fx_veh);
    total_moment_y += (arm_x * fy_veh) - (arm_y * fx_veh);
    total_self_aligning_moment += mz_tire;
  }

  // Fill the vehicle state structure
  vehicle_state.vx = state(VX);
  vehicle_state.vy = state(VY);
  vehicle_state.yaw_rate = state(YAW_RATE);
  vehicle_state.steering_angle = state(ST_ANGLE);
  vehicle_state.front_left_forces = tire_forces_cache_.segment<4>(FL * 4);
  vehicle_state.front_right_forces = tire_forces_cache_.segment<4>(FR * 4);
  vehicle_state.rear_left_forces = tire_forces_cache_.segment<4>(RL * 4);
  vehicle_state.rear_right_forces = tire_forces_cache_.segment<4>(RR * 4);
  vehicle_state.wheels_torque = {torques_cache_(0), torques_cache_(1), torques_cache_(2),
                                 torques_cache_(3)};
  vehicle_state.wheels_speed = {state(FL_WHEEL_SPEED), state(FR_WHEEL_SPEED), state(RL_WHEEL_SPEED),
                                state(RR_WHEEL_SPEED)};
  vehicle_state.wheels_vertical_load = {
      total_vertical_loads_cache_(0), total_vertical_loads_cache_(1),
      total_vertical_loads_cache_(2), total_vertical_loads_cache_(3)};
  vehicle_state.aero_drag = aero_forces_cache_(0);
  vehicle_state.aero_downforce = aero_forces_cache_(2);
  vehicle_state.total_force_x = total_fx;
  vehicle_state.total_force_y = total_fy;
  vehicle_state.moment_fx = total_moment_x;
  vehicle_state.moment_fy = total_moment_y;
  vehicle_state.self_aligning_moment = total_self_aligning_moment;
  vehicle_state.total_torque_z = total_torque;

  return vehicle_state;
}