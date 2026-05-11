#include "models/process/component_based_vehicle_model.hpp"

ComponentBasedVehicleModel::ComponentBasedVehicleModel(
    const std::shared_ptr<SEParameters>& parameters)
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
}

void ComponentBasedVehicleModel::predict(Eigen::Ref<State> state,
                                         common_lib::structures::ControlCommand control_command,
                                         double dt) {
  // Scale control command to torque (02 version, assuming single motor RWD)
  double throttle_input =
      control_command.throttle_rl * parameters_->car_parameters_->motor_parameters->max_peak_torque;

  // Calculate torque distribution using the transmission model
  common_lib::structures::Wheels wheel_speeds;
  wheel_speeds.front_left = state(FL_WHEEL_SPEED);
  wheel_speeds.front_right = state(FR_WHEEL_SPEED);
  wheel_speeds.rear_left = state(RL_WHEEL_SPEED);
  wheel_speeds.rear_right = state(RR_WHEEL_SPEED);

  common_lib::structures::Wheels torques_struct =
      transmission_model_->calculate_wheel_torques(throttle_input, wheel_speeds);
  Eigen::Vector4d torques(torques_struct.front_left, torques_struct.front_right,
                          torques_struct.rear_left, torques_struct.rear_right);

  // Calculate individual wheel yaw using the steering model
  Eigen::Vector4d wheel_angles = this->steering_model_->calculate_steering_angles(state(ST_ANGLE));

  // Calculate aerodynamic forces using the aero model
  Eigen::Vector3d aero_forces = aero_model_->aero_forces(state.segment<3>(VX));

  // Calculate load in each tire using the load transfer model and aero output
  LoadTransferInput load_transfer_input;
  load_transfer_input.longitudinal_acceleration = state(AX);
  load_transfer_input.lateral_acceleration = state(AY);
  load_transfer_input.downforce = aero_forces(2);
  common_lib::structures::Wheels load_distribution =
      load_transfer_model_->compute_loads(load_transfer_input);

  Eigen::Vector4d total_vertical_loads(load_distribution.front_left, load_distribution.front_right,
                                       load_distribution.rear_left, load_distribution.rear_right);

  // TIRE MODEL
  TireInput tire_input;
  Eigen::Matrix<double, 16, 1> tire_forces;  // 4 tires * 4 forces each
  tire_input.vx = state(VX);
  tire_input.vy = state(VY);
  tire_input.yaw_rate = state(YAW_RATE);
  for (Tire tire : {FL, FR, RL, RR}) {
    tire_input.tire = tire;
    tire_input.steering_angle = wheel_angles(tire);
    tire_input.wheel_angular_speed = state(FL_WHEEL_SPEED + tire);
    tire_input.vertical_load = total_vertical_loads(tire);
    tire_forces.segment<4>(tire * 4) =
        tire_model_->calculate_tire_forces_not_transient(tire_input);  //[Fx, Fy, My, Mz]
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
        (torques(tire) - tire_forces(tire * 4) * wheel_radius / inertia) *
        dt;  // No braking torque and no rolling resistance

    // Current tire forces in tire-local frame
    double fx_tire = tire_forces(tire * 4);
    double fy_tire = tire_forces(tire * 4 + 1);
    // double mz_tire = tire_forces(tire * 4 + 3);

    // Transform to vehicle frame
    double cos_delta = cos(wheel_angles(tire));
    double sin_delta = sin(wheel_angles(tire));

    double fx_veh = fx_tire * cos_delta - fy_tire * sin_delta;
    double fy_veh = fx_tire * sin_delta + fy_tire * cos_delta;

    total_fx += fx_veh;
    total_fy += fy_veh;

    // Calculate Moment arms
    double arm_x = (tire == FL || tire == FR) ? lf : -lr;
    double arm_y = (tire == FL || tire == RL) ? half_width : -half_width;

    // Sum moments
    total_torque += (arm_x * fy_veh) - (arm_y * fx_veh);
  }
  // RCLCPP_INFO_STREAM(rclcpp::get_logger("ComponentBasedVehicleModel"),"Torque command: " <<
  // throttle_input << " Total Fx: " << total_fx << " Total Fy: " << total_fy << " Total Torque: "
  // << total_torque);
  //  RCLCPP_INFO_STREAM(rclcpp::get_logger("ComponentBasedVehicleModel"),"Slip ratios: " <<
  //  tire_input.last_slip_ratio.transpose());

  // Compute accelerations
  double ax = total_fx / parameters_->car_parameters_->total_mass + state(VY) * state(YAW_RATE);

  double ay = total_fy / parameters_->car_parameters_->total_mass - state(VX) * state(YAW_RATE);

  // Integrate velocities using derived values
  state(VX) += ax * dt;
  state(VY) += ay * dt;

  // Store accelerations
  state(AX) = ax;
  state(AY) = ay;

  // Update Yaw Rate
  state(YAW_RATE) += (total_torque / parameters_->car_parameters_->Izz) * dt;
};

VehicleState ComponentBasedVehicleModel::get_process_model_data(
    const State& state, const common_lib::structures::ControlCommand& control_command) {
  VehicleState vehicle_state;

  // Scale control command to torque (02 version, assuming single motor RWD)
  double throttle_input =
      control_command.throttle_rl * parameters_->car_parameters_->motor_parameters->max_peak_torque;

  // Calculate torque distribution using the transmission model
  common_lib::structures::Wheels wheel_speeds;
  wheel_speeds.front_left = state(FL_WHEEL_SPEED);
  wheel_speeds.front_right = state(FR_WHEEL_SPEED);
  wheel_speeds.rear_left = state(RL_WHEEL_SPEED);
  wheel_speeds.rear_right = state(RR_WHEEL_SPEED);

  common_lib::structures::Wheels torques_struct =
      transmission_model_->calculate_wheel_torques(throttle_input, wheel_speeds);
  Eigen::Vector4d torques(torques_struct.front_left, torques_struct.front_right,
                          torques_struct.rear_left, torques_struct.rear_right);

  // Calculate individual wheel yaw using the steering model
  Eigen::Vector4d wheel_angles = this->steering_model_->calculate_steering_angles(state(ST_ANGLE));

  // Calculate aerodynamic forces using the aero model
  Eigen::Vector3d aero_forces = aero_model_->aero_forces(state.segment<3>(VX));

  // Calculate load in each tire using the load transfer model and aero output
  LoadTransferInput load_transfer_input;
  load_transfer_input.longitudinal_acceleration = state(AX);
  load_transfer_input.lateral_acceleration = state(AY);
  load_transfer_input.downforce = aero_forces(2);
  common_lib::structures::Wheels load_distribution =
      load_transfer_model_->compute_loads(load_transfer_input);

  Eigen::Vector4d total_vertical_loads(load_distribution.front_left, load_distribution.front_right,
                                       load_distribution.rear_left, load_distribution.rear_right);

  // TIRE MODEL
  TireInput tire_input;
  Eigen::Matrix<double, 16, 1>
      tire_forces;  // 4 tires * 4 forces each, fixed-size avoids heap alloc
  tire_input.vx = state(VX);
  tire_input.vy = state(VY);
  tire_input.yaw_rate = state(YAW_RATE);
  for (Tire tire : {FL, FR, RL, RR}) {
    tire_input.tire = tire;
    tire_input.steering_angle = wheel_angles(tire);
    tire_input.wheel_angular_speed = state(FL_WHEEL_SPEED + tire);
    tire_input.vertical_load = total_vertical_loads(tire);
    tire_forces.segment<4>(tire * 4) =
        tire_model_->calculate_tire_forces_not_transient(tire_input);  //[Fx, Fy, My, Mz]
  }

  double total_fx = aero_forces(0);
  double total_fy = aero_forces(1);
  double total_torque = 0.0;
  double total_moment_x = 0.0;
  double total_moment_y = 0.0;
  double total_self_aligning_moment = 0.0;
  double lr = parameters_->car_parameters_->cg_2_rear_axis;  // distance from CG to rear axle
  double lf = parameters_->car_parameters_->wheelbase - lr;  // distance from CG to front axle
  double half_width = parameters_->car_parameters_->track_width *
                      0.5;  // half of the track width for moment arm calculations

  for (Tire tire : {FL, FR, RL, RR}) {
    // Current tire forces in tire-local frame
    double fx_tire = tire_forces(tire * 4);
    double fy_tire = tire_forces(tire * 4 + 1);
    double mz_tire = tire_forces(tire * 4 + 3);

    // Transform to vehicle frame
    double cos_delta = cos(wheel_angles(tire));
    double sin_delta = sin(wheel_angles(tire));

    double fx_veh = fx_tire * cos_delta - fy_tire * sin_delta;
    double fy_veh = fx_tire * sin_delta + fy_tire * cos_delta;

    total_fx += fx_veh;
    total_fy += fy_veh;

    // Calculate Moment arms
    double arm_x = (tire == FL || tire == FR) ? lf : -lr;
    double arm_y = (tire == FL || tire == RL) ? half_width : -half_width;

    // Sum moments
    total_torque += (arm_x * fy_veh) - (arm_y * fx_veh) + mz_tire;
    total_moment_x += (arm_y * fy_veh) + (arm_x * fx_veh);
    total_moment_y += (arm_x * fy_veh) - (arm_y * fx_veh);
    total_self_aligning_moment += mz_tire;
  }

  // Fill the vehicle state structure
  vehicle_state.vx = state(VX);
  vehicle_state.vy = state(VY);
  vehicle_state.yaw_rate = state(YAW_RATE);
  vehicle_state.steering_angle = state(ST_ANGLE);
  vehicle_state.front_left_forces = tire_forces.segment<4>(FL * 4);
  vehicle_state.front_right_forces = tire_forces.segment<4>(FR * 4);
  vehicle_state.rear_left_forces = tire_forces.segment<4>(RL * 4);
  vehicle_state.rear_right_forces = tire_forces.segment<4>(RR * 4);
  vehicle_state.wheels_torque = {torques(0), torques(1), torques(2), torques(3)};
  vehicle_state.wheels_speed = {state(FL_WHEEL_SPEED), state(FR_WHEEL_SPEED), state(RL_WHEEL_SPEED),
                                state(RR_WHEEL_SPEED)};
  vehicle_state.wheels_vertical_load = {total_vertical_loads(0), total_vertical_loads(1),
                                        total_vertical_loads(2), total_vertical_loads(3)};
  vehicle_state.aero_drag = aero_forces(0);
  vehicle_state.aero_downforce = aero_forces(2);
  vehicle_state.total_force_x = total_fx;
  vehicle_state.total_force_y = total_fy;
  vehicle_state.wheels_slip_ratio.front_left = tire_input.last_slip_ratio(FL);
  vehicle_state.wheels_slip_ratio.front_right = tire_input.last_slip_ratio(FR);
  vehicle_state.wheels_slip_ratio.rear_left = tire_input.last_slip_ratio(RL);
  vehicle_state.wheels_slip_ratio.rear_right = tire_input.last_slip_ratio(RR);
  vehicle_state.wheels_slip_angle.front_left = tire_input.last_slip_angle(FL);
  vehicle_state.wheels_slip_angle.front_right = tire_input.last_slip_angle(FR);
  vehicle_state.wheels_slip_angle.rear_left = tire_input.last_slip_angle(RL);
  vehicle_state.wheels_slip_angle.rear_right = tire_input.last_slip_angle(RR);
  vehicle_state.moment_fx = total_moment_x;
  vehicle_state.moment_fy = total_moment_y;
  vehicle_state.self_aligning_moment = total_self_aligning_moment;
  vehicle_state.total_torque_z = total_torque;

  return vehicle_state;
}