#include "vehicle_model/state_est_model.hpp"

StateEstModel::StateEstModel(const InvictaSimParameters& simulator_parameters)
    : VehicleModel(simulator_parameters) {
  this->tire_model_ = tire_models_map.at(simulator_parameters.tire_model.c_str())(
      simulator_parameters.car_parameters);
  this->motor_ = motor_models_map.at(simulator_parameters.motor_model.c_str())(
      simulator_parameters.car_parameters);
  this->battery_ = battery_models_map.at(simulator_parameters.battery_model.c_str())(
      simulator_parameters.car_parameters);
  this->steering_motor_ = steering_motor_models_map.at(
      simulator_parameters.steering_motor_model.c_str())(simulator_parameters.car_parameters);
  this->transmission_ = transmission_models_map.at(simulator_parameters.transmission_model.c_str())(
      simulator_parameters.car_parameters);
  this->aero_ = aero_models_map.at(simulator_parameters.aero_model.c_str())(
      simulator_parameters.car_parameters);
  this->load_transfer_ = load_transfer_models_map.at(
      simulator_parameters.load_transfer_model.c_str())(simulator_parameters.car_parameters);
  this->steering_ = steering_models_map.at(simulator_parameters.steering_model.c_str())(
      simulator_parameters.car_parameters);
}

void StateEstModel::step(double dt, common_lib::structures::Wheels throttle, double angle) {
  using Clock = std::chrono::steady_clock;

  // Motor + battery
  const auto powertrain_start = Clock::now();
  double motor_torque = throttle.rear_left * car_parameters_->motor_parameters->max_peak_torque;
  const auto powertrain_end = Clock::now();

  // Calculate torque distribution using the transmission model
  const auto transmission_start = Clock::now();
  state_->wheels_torque =
      transmission_->calculate_wheel_torques(motor_torque, state_->wheels_speed);
  const auto transmission_end = Clock::now();
  Eigen::Vector4d torques(state_->wheels_torque.front_left, state_->wheels_torque.front_right,
                          state_->wheels_torque.rear_left, state_->wheels_torque.rear_right);

  // Update steering angle first so tire forces use the latest actuator state.
  const auto steering_start = Clock::now();
  double steering_rate = steering_motor_->compute_steering_rate(state_->steering_angle, angle);
  state_->steering_angle += steering_rate * dt;
  const auto steering_end = Clock::now();

  // Calculate individual wheel yaw using the steering model
  Eigen::Vector4d wheel_angles = this->steering_->calculate_steering_angles(state_->steering_angle);

  // Calculate aerodynamic forces using the aero model
  const auto aero_start = Clock::now();
  Eigen::Vector3d aero_forces =
      aero_->aero_forces(Eigen::Vector3d(state_->vx, state_->vy, state_->yaw_rate));
  const auto aero_end = Clock::now();
  state_->aero_drag = aero_forces(0);
  state_->aero_downforce = aero_forces(2);

  // Calculate load in each tire using the load transfer model and aero output
  const auto load_transfer_start = Clock::now();
  LoadTransferInput load_transfer_input;
  load_transfer_input.longitudinal_acceleration = state_->ax;
  load_transfer_input.lateral_acceleration = state_->ay;
  load_transfer_input.downforce = aero_forces(2);
  common_lib::structures::Wheels load_distribution =
      load_transfer_->compute_loads(load_transfer_input);
  const auto load_transfer_end = Clock::now();
  state_->wheels_vertical_load = load_distribution;

  Eigen::Vector4d total_vertical_loads(load_distribution.front_left, load_distribution.front_right,
                                       load_distribution.rear_left, load_distribution.rear_right);

  // TIRE MODEL
  Eigen::Vector4d wheel_speeds(state_->wheels_speed.front_left, state_->wheels_speed.front_right,
                               state_->wheels_speed.rear_left, state_->wheels_speed.rear_right);
  TireInput tire_input;
  const auto tire_start = Clock::now();
  Eigen::VectorXd tire_forces = Eigen::VectorXd(16);  // 4 tires * 4 forces each
  Eigen::Vector4d slip_angles = Eigen::Vector4d::Zero();
  tire_input.vx = state_->vx;
  tire_input.vy = state_->vy;
  tire_input.yaw_rate = state_->yaw_rate;
  tire_input.dt = dt;
  tire_input.last_slip_ratio =
      Eigen::Vector4d(state_->wheels_slip_ratio.front_left, state_->wheels_slip_ratio.front_right,
                      state_->wheels_slip_ratio.rear_left, state_->wheels_slip_ratio.rear_right);
  for (Tire tire : {FL, FR, RL, RR}) {
    tire_input.tire = tire;
    tire_input.steering_angle = wheel_angles(tire);
    tire_input.wheel_angular_speed = wheel_speeds(tire);
    tire_input.vertical_load = total_vertical_loads(tire);
    tire_forces.segment<4>(tire * 4) =
        tire_model_->calculate_tire_forces_not_transient(tire_input);  //[Fx, Fy, My, Mz]
    slip_angles(tire) = tire_input.slip_angle;
  }

  // Info for simulator publishers
  state_->wheels_slip_angle = common_lib::structures::Wheels(slip_angles(0), slip_angles(1),
                                                             slip_angles(2), slip_angles(3));
  state_->front_left_forces = tire_forces.segment<4>(FL * 4);
  state_->front_right_forces = tire_forces.segment<4>(FR * 4);
  state_->rear_left_forces = tire_forces.segment<4>(RL * 4);
  state_->rear_right_forces = tire_forces.segment<4>(RR * 4);
  state_->wheels_slip_ratio =
      common_lib::structures::Wheels(tire_input.last_slip_ratio(0), tire_input.last_slip_ratio(1),
                                     tire_input.last_slip_ratio(2), tire_input.last_slip_ratio(3));
  const auto tire_end = Clock::now();

  // Update state using the calculated values

  double total_fx = aero_forces(0);
  double total_fy = aero_forces(1);
  double moment_fy = 0.0;
  double moment_fx = 0.0;
  double self_aligning_moment = 0.0;
  double total_torque = 0.0;
  double lr = car_parameters_->cg_2_rear_axis;  // distance from CG to rear axle
  double lf = car_parameters_->wheelbase - lr;  // distance from CG to front axle
  double half_width =
      car_parameters_->track_width * 0.5;  // half of the track width for moment arm calculations
  double wheel_radius = car_parameters_->tire_parameters->effective_tire_r;
  double inertia = car_parameters_->tire_parameters->wheel_inertia;

  for (Tire tire : {FL, FR, RL, RR}) {
    double sign_r = 2.0 / M_PI * std::atan(10.0 * wheel_speeds(tire));
    // Update wheel speeds using the calculated torques and tire forces
    wheel_speeds(tire) += ((torques(tire) - tire_forces(tire * 4) * wheel_radius -
                            tire_forces(tire * 4 + 2) * sign_r) /
                           inertia) *
                          dt;  // No braking torque

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
    moment_fy += arm_x * fy_veh;
    moment_fx += -arm_y * fx_veh;
    self_aligning_moment += mz_tire;
    total_torque += (arm_x * fy_veh) - (arm_y * fx_veh) + mz_tire;
  }

  state_->wheels_speed = common_lib::structures::Wheels(wheel_speeds(0), wheel_speeds(1),
                                                        wheel_speeds(2), wheel_speeds(3));

  // Update Accelerations
  double total_ax = total_fx / car_parameters_->total_mass + state_->vy * state_->yaw_rate;
  double total_ay = total_fy / car_parameters_->total_mass - state_->vx * state_->yaw_rate;
  state_->total_force_x = total_fx;
  state_->total_force_y = total_fy;
  state_->moment_fy = moment_fy;
  state_->moment_fx = moment_fx;
  state_->self_aligning_moment = self_aligning_moment;
  state_->total_torque_z = total_torque;

  // Trapezoidal integration for velocity
  state_->vx += 0.5 * (total_ax + state_->ax) * dt;
  state_->vy += 0.5 * (total_ay + state_->ay) * dt;
  state_->ax = total_ax;
  state_->ay = total_ay;

  // Update Yaw Rate
  state_->yaw_rate += (total_torque / car_parameters_->Izz) * dt;

  // Integrate heading and position in global coordinates.
  state_->yaw += state_->yaw_rate * dt;
  if (state_->yaw > M_PI) {
    state_->yaw -= 2.0 * M_PI;
  }
  if (state_->yaw < -M_PI) {
    state_->yaw += 2.0 * M_PI;
  }

  const double cos_yaw = cos(state_->yaw);
  const double sin_yaw = sin(state_->yaw);
  const double v_global_x = state_->vx * cos_yaw - state_->vy * sin_yaw;
  const double v_global_y = state_->vx * sin_yaw + state_->vy * cos_yaw;
  state_->x += v_global_x * dt;
  state_->y += v_global_y * dt;

  // Per-subsystem execution times in milliseconds.
  execution_times_->powertrain_ms =
      std::chrono::duration<double, std::milli>(powertrain_end - powertrain_start).count();
  execution_times_->transmission_ms =
      std::chrono::duration<double, std::milli>(transmission_end - transmission_start).count();
  execution_times_->aero_ms =
      std::chrono::duration<double, std::milli>(aero_end - aero_start).count();
  execution_times_->steering_ms =
      std::chrono::duration<double, std::milli>(steering_end - steering_start).count();
  execution_times_->load_transfer_ms =
      std::chrono::duration<double, std::milli>(load_transfer_end - load_transfer_start).count();
  execution_times_->tire_ms =
      std::chrono::duration<double, std::milli>(tire_end - tire_start).count();
}

void StateEstModel::reset() {
  state_ = std::make_shared<VehicleState>();
  *execution_times_ = VehicleModelExecutionTimes{};
}

std::string StateEstModel::get_model_name() const { return "StateEstModel"; }

double StateEstModel::calculate_powertrain_torque(double throttle_input, double dt) {
  double avg_wheel_speed = (state_->wheels_speed.rear_left + state_->wheels_speed.rear_right) / 2.0;
  double motor_omega = avg_wheel_speed * car_parameters_->gear_ratio;
  double motor_rpm = (motor_omega * 60.0 / (2.0 * M_PI));

  // Calculate max torque at current RPM.
  double max_motor_torque = motor_->get_max_torque_at_rpm(motor_rpm);
  double reference_motor_torque = throttle_input * max_motor_torque;

  // Motor efficiency at this state.
  double motor_efficiency = motor_->get_efficiency(std::abs(reference_motor_torque), motor_rpm);

  // Corresponding current request for the desired torque, always positive.
  double requested_motor_current =
      std::abs(reference_motor_torque) /
      (car_parameters_->motor_parameters->kt_constant * std::max(motor_efficiency, 0.05));

  // Calculate the allowed current from the battery.
  double allowed_motor_current = battery_->calculate_allowed_current(requested_motor_current);

  // Actual motor torque limited by the battery.
  double actual_motor_torque =
      allowed_motor_current * car_parameters_->motor_parameters->kt_constant * motor_efficiency;

  // Restore the sign of the torque.
  if (reference_motor_torque < 0) {
    actual_motor_torque *= -1.0;
  }

  battery_->update_state(allowed_motor_current, dt);
  motor_->update_state(allowed_motor_current, actual_motor_torque, dt);

  state_->motor_torque = actual_motor_torque;
  state_->motor_omega = motor_omega;
  state_->motor_current = motor_->get_current();
  state_->motor_thermal_state = motor_->get_thermal_state();
  state_->motor_thermal_capacity = motor_->get_thermal_capacity();
  state_->battery_current = battery_->get_current();
  state_->battery_voltage = battery_->get_voltage();
  state_->battery_soc = battery_->get_soc();
  state_->battery_open_circuit_voltage = battery_->get_open_circuit_voltage();

  return actual_motor_torque;
}
