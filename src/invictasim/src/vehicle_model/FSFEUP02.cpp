#include "vehicle_model/FSFEUP02.hpp"

FSFEUP02Model::FSFEUP02Model(const InvictaSimParameters& simulator_parameters)
    : VehicleModel(simulator_parameters) {
  this->tire_model_ = tire_models_map.at(simulator_parameters.tire_model.c_str())(
      simulator_parameters.car_parameters);
  this->motor_ = motor_models_map.at(simulator_parameters.motor_model.c_str())(
      simulator_parameters.car_parameters);
  this->battery_ = battery_models_map.at(simulator_parameters.battery_model.c_str())(
      simulator_parameters.car_parameters);
  this->transmission_ = transmission_models_map.at(simulator_parameters.transmission_model.c_str())(
      simulator_parameters.car_parameters);
  this->inverter_ = inverter_models_map.at(simulator_parameters.inverter_model.c_str())(
      simulator_parameters.car_parameters);
  this->brake_ = brake_models_map.at(simulator_parameters.brake_model.c_str())(
      simulator_parameters.car_parameters);
  this->aero_ = aero_models_map.at(simulator_parameters.aero_model.c_str())(
      simulator_parameters.car_parameters);
  this->load_transfer_ = load_transfer_models_map.at(
      simulator_parameters.load_transfer_model.c_str())(simulator_parameters.car_parameters);
  this->steering_ = steering_models_map.at(simulator_parameters.steering_model.c_str())(
      simulator_parameters.car_parameters);
  this->steering_motor_ = steering_motor_models_map.at(
      simulator_parameters.steering_motor_model.c_str())(simulator_parameters.car_parameters);
  this->control_mode_ = simulator_parameters.control_mode;
}

void FSFEUP02Model::step(double dt, common_lib::structures::Wheels throttle, double angle) {
  // The driven-wheel / tyre-slip mode is stiff (time constant ~ I*denom/(kxk*r^2),
  // sub-millisecond at low speed), so a single explicit RK4 step at the incoming
  // dt is numerically unstable near launch and causes runaway wheel spin.  The
  // stable step scales with speed, so we substep adaptively: one step at cruise,
  // finer steps only in the first low-speed instants.  This mirrors the offline
  // tuning model (inline_02.hpp) so tuned parameters transfer to the live sim.
  if (!(dt > 0.0)) return;
  constexpr double kBaseSubDt = 2.0e-4;  // stable step at ~1 m/s
  constexpr int kMaxSub = 64;
  const double speed = std::sqrt(state_->vx * state_->vx + state_->vy * state_->vy);
  const double stable_dt = kBaseSubDt * std::max(1.0, speed);
  const int n_sub = std::min(kMaxSub, std::max(1, static_cast<int>(std::ceil(dt / stable_dt))));
  const double sub_dt = dt / static_cast<double>(n_sub);
  for (int k = 0; k < n_sub; ++k) integrate_substep(sub_dt, throttle, angle);
}

void FSFEUP02Model::integrate_substep(double dt, common_lib::structures::Wheels throttle, double angle) {
  using Clock = std::chrono::steady_clock;

  const auto powertrain_start = Clock::now();
  const double throttle_input =
      (throttle.rear_left + throttle.rear_right) / 2.0;  // Average throttle for rear-wheel drive

  common_lib::structures::Wheels brake_torques;
  double inverter_command = throttle_input;
  if (control_mode_ == "manual" && throttle_input < 0.0) {
    brake_torques = brake_->calculate_brake_torques(-throttle_input);
    inverter_command = 0.0;
  }

  const bool braking = brake_torques.front_left > 0.0 || brake_torques.front_right > 0.0 ||
                       brake_torques.rear_left > 0.0 || brake_torques.rear_right > 0.0;
  const double motor_input = inverter_->calculate_inverter_throttle(inverter_command, dt);
  const double motor_torque =
      calculate_powertrain_torque(motor_input, dt);
  const auto powertrain_end = Clock::now();

  VehicleModelExecutionTimes current_times;
  current_times.powertrain_ms =
      std::chrono::duration<double, std::milli>(powertrain_end - powertrain_start).count();

  const auto integration_start = Clock::now();
  StateVec s = StateVec::Zero();
  s(VX) = state_->vx;
  s(VY) = state_->vy;
  s(YAW_RATE) = state_->yaw_rate;
  s(YAW) = state_->yaw;
  s(PX) = state_->x;
  s(PY) = state_->y;
  s(ST_ANGLE) = state_->steering_angle;
  s(FL_W) = state_->wheels_speed.front_left;
  s(FR_W) = state_->wheels_speed.front_right;
  s(RL_W) = state_->wheels_speed.rear_left;
  s(RR_W) = state_->wheels_speed.rear_right;
  s(AX) = state_->ax;
  s(AY) = state_->ay;

  const StateVec k1 =
      get_state_derivative(s, motor_torque, brake_torques, angle, dt, false, &current_times);
  const StateVec k2 = get_state_derivative(s + 0.5 * dt * k1, motor_torque, brake_torques, angle,
                                           dt, false, &current_times);
  const StateVec k3 = get_state_derivative(s + 0.5 * dt * k2, motor_torque, brake_torques, angle,
                                           dt, false, &current_times);
  const StateVec k4 = get_state_derivative(s + dt * k3, motor_torque, brake_torques, angle, dt,
                                           false, &current_times);
  StateVec s_next = s + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);

  state_->vx = s_next(VX);
  state_->vy = s_next(VY);
  state_->yaw_rate = s_next(YAW_RATE);
  state_->yaw = s_next(YAW);
  state_->x = s_next(PX);
  state_->y = s_next(PY);
  state_->steering_angle = s_next(ST_ANGLE);
  state_->wheels_speed =
      common_lib::structures::Wheels(s_next(FL_W), s_next(FR_W), s_next(RL_W), s_next(RR_W));
  state_->ax = s_next(AX);
  state_->ay = s_next(AY);

  if (state_->ebs_active) {
    state_->wheels_speed = {0.0, 0.0, 0.0, 0.0};
  }

  // Prevent oscillations at very low speeds by forcing a dead stop
  double speed = std::sqrt(state_->vx * state_->vx + state_->vy * state_->vy);
  if (speed < 0.05 && (std::abs(throttle_input) < 0.01 || braking)) {
    state_->vx = 0.0;
    state_->vy = 0.0;
    state_->ax = 0.0;
    state_->ay = 0.0;
    state_->yaw_rate = 0.0;
    state_->wheels_speed.front_left = 0.0;
    state_->wheels_speed.front_right = 0.0;
    state_->wheels_speed.rear_left = 0.0;
    state_->wheels_speed.rear_right = 0.0;
    state_->wheels_slip_ratio = {0.0, 0.0, 0.0, 0.0};
    state_->wheels_slip_angle = {0.0, 0.0, 0.0, 0.0};
  }

  // Keep yaw within [-pi, pi]
  if (state_->yaw > M_PI) {
    state_->yaw -= 2.0 * M_PI;
  }
  if (state_->yaw < -M_PI) {
    state_->yaw += 2.0 * M_PI;
  }

  s_next(VX) = state_->vx;
  s_next(VY) = state_->vy;
  s_next(YAW_RATE) = state_->yaw_rate;
  s_next(YAW) = state_->yaw;
  s_next(PX) = state_->x;
  s_next(PY) = state_->y;
  s_next(ST_ANGLE) = state_->steering_angle;
  s_next(FL_W) = state_->wheels_speed.front_left;
  s_next(FR_W) = state_->wheels_speed.front_right;
  s_next(RL_W) = state_->wheels_speed.rear_left;
  s_next(RR_W) = state_->wheels_speed.rear_right;
  s_next(AX) = state_->ax;
  s_next(AY) = state_->ay;
  get_state_derivative(s_next, motor_torque, brake_torques, angle, dt, true, &current_times);

  const auto integration_end = Clock::now();

  current_times.integration_ms =
      std::chrono::duration<double, std::milli>(integration_end - integration_start).count();
  *execution_times_ = current_times;
}

double FSFEUP02Model::calculate_powertrain_torque(double throttle_input, double dt) {
  double motor_omega = transmission_->calculate_motor_omega(state_->wheels_speed);
  double motor_rpm = std::abs(motor_omega * 60.0f / (2.0f * M_PI));

  // Calculate max motor torque at current RPM
  double max_motor_torque = motor_->get_max_torque_at_rpm(motor_rpm);
  double reference_motor_torque = throttle_input * max_motor_torque;

  // Avoid zero power request at launch
  const double min_omega_for_power = 10.0;  // rad/s,

  double omega_sign_source =
      std::abs(motor_omega) > 1e-3 ? motor_omega : reference_motor_torque;

  double omega_for_power =
      std::copysign(std::max(std::abs(motor_omega), min_omega_for_power),
                    omega_sign_source);

  // Mechanical power request
  double mechanical_power_request =
      reference_motor_torque * omega_for_power;

  // Mechanical-Eletrical efficiency
  double motor_efficiency = motor_->get_efficiency(std::abs(reference_motor_torque), motor_rpm);
  double inverter_efficiency = inverter_->get_efficiency();
  double total_efficiency = motor_efficiency * inverter_efficiency;

  // Requested battery power
  double battery_power_request;
  if (mechanical_power_request >= 0.0) {
    // Normal
    battery_power_request = mechanical_power_request / total_efficiency;
  } else {
    // Regen
    battery_power_request = mechanical_power_request * total_efficiency;
  }

  // Battery current limit
  double battery_voltage = battery_->get_voltage();
  double requested_battery_current =
      battery_power_request / battery_voltage;
  double allowed_battery_current =
      battery_->calculate_allowed_current(requested_battery_current);

  // Provided battery power
  double allowed_battery_voltage =
      battery_->get_voltage(allowed_battery_current);
  double allowed_battery_power =
      allowed_battery_current * allowed_battery_voltage;

  // Convert back to available mechanical power
  double allowed_mechanical_power;
  if (allowed_battery_power >= 0.0) {
    allowed_mechanical_power = allowed_battery_power * total_efficiency;
  } else {
    allowed_mechanical_power = allowed_battery_power / total_efficiency;
  }

  double actual_motor_torque = std::copysign(
      std::min(std::abs(allowed_mechanical_power / omega_for_power),
               std::abs(reference_motor_torque)),
      reference_motor_torque);

  // Motor phase current magnitude. Sign follows the battery current convention:
  // positive while discharging, negative while charging/regenerating.
  double motor_phase_current =
      std::abs(actual_motor_torque) / car_parameters_->motor_parameters->kt_constant;

  // Inverter phase current limit
  double max_inverter_phase_current =
      car_parameters_->inverter_parameters->max_phase_current;
  if (motor_phase_current > max_inverter_phase_current) {
    double limited_torque =
        max_inverter_phase_current *
        car_parameters_->motor_parameters->kt_constant;

    actual_motor_torque =
        std::copysign(limited_torque, actual_motor_torque);

    motor_phase_current = max_inverter_phase_current;
  }

  const double signed_motor_phase_current =
      std::abs(allowed_battery_current) > 1e-9
          ? std::copysign(motor_phase_current, allowed_battery_current)
          : 0.0;

  // Update models
  battery_->update_state(allowed_battery_current, dt);
  motor_->update_state(signed_motor_phase_current,
                       actual_motor_torque,
                       dt);

  state_->motor_torque = actual_motor_torque;
  state_->motor_omega = motor_omega;
  state_->motor_current = motor_->get_current();
  state_->motor_thermal_state = motor_->get_thermal_state();
  state_->motor_thermal_capacity = motor_->get_thermal_capacity();
  state_->battery_current = battery_->get_current();
  state_->battery_voltage = battery_->get_voltage();
  state_->battery_soc = battery_->get_soc();
  state_->battery_open_circuit_voltage = battery_->get_open_circuit_voltage();

  return static_cast<double>(actual_motor_torque);
}

FSFEUP02Model::StateVec FSFEUP02Model::get_state_derivative(
    const StateVec& s, double motor_torque, const common_lib::structures::Wheels& brake_torques,
    double steering_target, double dt, bool write_telemetry,
    VehicleModelExecutionTimes* execution_times) {
  using Clock = std::chrono::steady_clock;
  StateVec ds = StateVec::Zero();

  const common_lib::structures::Wheels wheel_speeds(s(FL_W), s(FR_W), s(RL_W), s(RR_W));

  const auto transmission_start = Clock::now();
  const common_lib::structures::Wheels wheel_torques =
      transmission_->calculate_wheel_torques(motor_torque, wheel_speeds);
  const auto transmission_end = Clock::now();
  execution_times->transmission_ms +=
      std::chrono::duration<double, std::milli>(transmission_end - transmission_start).count();
  const Eigen::Vector4d torques(wheel_torques.front_left, wheel_torques.front_right,
                                wheel_torques.rear_left, wheel_torques.rear_right);

  const auto steering_start = Clock::now();
  Eigen::Vector4d wheel_angles = steering_->calculate_steering_angles(s(ST_ANGLE));
  const auto steering_angles_end = Clock::now();
  execution_times->steering_ms +=
      std::chrono::duration<double, std::milli>(steering_angles_end - steering_start).count();

  const auto aero_start = Clock::now();
  const Eigen::Vector3d aero_forces =
      aero_->aero_forces(Eigen::Vector3d(s(VX), s(VY), s(YAW_RATE)));
  const auto aero_end = Clock::now();
  execution_times->aero_ms +=
      std::chrono::duration<double, std::milli>(aero_end - aero_start).count();

  const auto load_transfer_start = Clock::now();
  const common_lib::structures::Wheels load_distribution = load_transfer_->compute_loads(
      LoadTransferInput{s(AX), s(AY), aero_forces[2]});  // aero_forces{Fx,Fy,Fz}
  const auto load_transfer_end = Clock::now();
  execution_times->load_transfer_ms +=
      std::chrono::duration<double, std::milli>(load_transfer_end - load_transfer_start).count();
  const Eigen::Vector4d vertical_loads(load_distribution.front_left, load_distribution.front_right,
                                       load_distribution.rear_left, load_distribution.rear_right);

  const auto tire_start = Clock::now();
  Eigen::VectorXd tire_forces(16);
  Eigen::Vector4d slip_ratio = Eigen::Vector4d::Zero();
  Eigen::Vector4d slip_angle = Eigen::Vector4d::Zero();
  Eigen::Vector4d contact_patch_longitudinal_velocity = Eigen::Vector4d::Zero();
  TireInput tire_input;
  tire_input.dt = dt;
  tire_input.vx = s(VX);
  tire_input.vy = s(VY);
  tire_input.yaw_rate = s(YAW_RATE);
  tire_input.last_slip_ratio =
      Eigen::Vector4d(state_->wheels_slip_ratio.front_left, state_->wheels_slip_ratio.front_right,
                      state_->wheels_slip_ratio.rear_left, state_->wheels_slip_ratio.rear_right);
  tire_input.last_slip_angle =
      Eigen::Vector4d(state_->wheels_slip_angle.front_left, state_->wheels_slip_angle.front_right,
                      state_->wheels_slip_angle.rear_left, state_->wheels_slip_angle.rear_right);

  for (Tire tire : {FL, FR, RL, RR}) {
    tire_input.tire = tire;
    tire_input.steering_angle = wheel_angles(tire);
    tire_input.wheel_angular_speed = s(FL_W + tire);
    tire_input.vertical_load = vertical_loads(tire);
    tire_forces.segment<4>(tire * 4) = tire_model_->calculate_tire_forces(tire_input);
    slip_ratio(tire) = tire_input.slip_ratio;
    slip_angle(tire) = tire_input.slip_angle;
    contact_patch_longitudinal_velocity(tire) = tire_input.vcx;
  }
  const auto tire_end = Clock::now();
  execution_times->tire_ms +=
      std::chrono::duration<double, std::milli>(tire_end - tire_start).count();

  double total_fx = aero_forces[0];
  double total_fy = aero_forces[1];
  double moment_fy = 0.0;
  double moment_fx = 0.0;
  double self_aligning_moment = 0.0;
  double total_torque = 0.0;

  const double lr = car_parameters_->cg_2_rear_axis;
  const double lf = car_parameters_->wheelbase - lr;
  const double half_width = car_parameters_->track_width / 2.0;
  const double wheel_radius = car_parameters_->tire_parameters->effective_tire_r;
  const Eigen::Vector4d brake_torques_by_tire(brake_torques.front_left,
                                              brake_torques.front_right,
                                              brake_torques.rear_left,
                                              brake_torques.rear_right);

  for (Tire tire : {FL, FR, RL, RR}) {
    const double fx_tire = tire_forces(tire * 4);
    const double fy_tire = tire_forces(tire * 4 + 1);
    const double mz_tire = tire_forces(tire * 4 + 3);
    const double cos_delta = std::cos(wheel_angles(tire));
    const double sin_delta = std::sin(wheel_angles(tire));

    const double fx_vehicle = fx_tire * cos_delta - fy_tire * sin_delta;
    const double fy_vehicle = fx_tire * sin_delta + fy_tire * cos_delta;
    const double arm_x = (tire == FL || tire == FR) ? lf : -lr;
    const double arm_y = (tire == FL || tire == RL) ? half_width : -half_width;

    total_fx += fx_vehicle;
    total_fy += fy_vehicle;
    moment_fy += arm_x * fy_vehicle;
    moment_fx += -arm_y * fx_vehicle;
    self_aligning_moment += mz_tire;
    total_torque += arm_x * fy_vehicle - arm_y * fx_vehicle + mz_tire;
  }

  const double ax = total_fx / car_parameters_->total_mass + s(VY) * s(YAW_RATE);
  const double ay = total_fy / car_parameters_->total_mass - s(VX) * s(YAW_RATE);
  ds(VX) = ax;
  ds(VY) = ay;
  ds(AX) = ax - s(AX);
  ds(AY) = ay - s(AY);
  ds(YAW_RATE) = total_torque / car_parameters_->Izz;
  ds(YAW) = s(YAW_RATE);
  ds(PX) = s(VX) * std::cos(s(YAW)) - s(VY) * std::sin(s(YAW));
  ds(PY) = s(VX) * std::sin(s(YAW)) + s(VY) * std::cos(s(YAW));
  const auto steering_motor_start = Clock::now();
  ds(ST_ANGLE) = steering_motor_->compute_steering_rate(s(ST_ANGLE), steering_target);
  const auto steering_motor_end = Clock::now();
  execution_times->steering_ms +=
      std::chrono::duration<double, std::milli>(steering_motor_end - steering_motor_start).count();

  if (state_->ebs_active) {
    ds(FL_W) = -s(FL_W) / std::max(dt, 1e-6);
    ds(FR_W) = -s(FR_W) / std::max(dt, 1e-6);
    ds(RL_W) = -s(RL_W) / std::max(dt, 1e-6);
    ds(RR_W) = -s(RR_W) / std::max(dt, 1e-6);
  } else {
    // Front: tire + rim inertia only; Rear: tire + rim + motor + transmission (reflected)
    const double front_inertia = car_parameters_->front_wheel_inertia;
    const double rear_inertia = car_parameters_->rear_wheel_inertia;
    for (Tire tire : {FL, FR, RL, RR}) {
      const double wheel_omega = s(FL_W + tire);
      double net_torque =
          torques(tire) - tire_forces(tire * 4) * wheel_radius - tire_forces(tire * 4 + 2);
      const double brake_sign = 2.0 / M_PI * std::atan(10.0 * wheel_omega);
      const double brake_torque = brake_torques_by_tire(tire);
      net_torque -= brake_torque * brake_sign;

      if (tire == FL || tire == FR) {
        net_torque -= car_parameters_->front_bearing_drag * wheel_omega;
      }

      const double inertia = (tire == FL || tire == FR) ? front_inertia : rear_inertia;
      const double wheel_acceleration = net_torque / inertia;
      if (brake_torque > 0.0 && wheel_omega * (wheel_omega + wheel_acceleration * dt) <= 0.0) {
        ds(FL_W + tire) = -wheel_omega / std::max(dt, 1e-6);
      } else {
        ds(FL_W + tire) = wheel_acceleration;
      }
    }
  }

  if (write_telemetry) {
    state_->front_left_forces = tire_forces.segment<4>(FL * 4);
    state_->front_right_forces = tire_forces.segment<4>(FR * 4);
    state_->rear_left_forces = tire_forces.segment<4>(RL * 4);
    state_->rear_right_forces = tire_forces.segment<4>(RR * 4);
    state_->wheels_torque = wheel_torques;
    state_->wheels_vertical_load = load_distribution;
    state_->wheels_slip_ratio = common_lib::structures::Wheels(slip_ratio(FL), slip_ratio(FR),
                                                               slip_ratio(RL), slip_ratio(RR));
    state_->wheels_slip_angle = common_lib::structures::Wheels(slip_angle(FL), slip_angle(FR),
                                                               slip_angle(RL), slip_angle(RR));
    state_->aero_drag = aero_forces[0];
    state_->aero_downforce = aero_forces[2];
    state_->total_force_x = total_fx;
    state_->total_force_y = total_fy;
    state_->moment_fy = moment_fy;
    state_->moment_fx = moment_fx;
    state_->self_aligning_moment = self_aligning_moment;
    state_->total_torque_z = total_torque;
  }

  return ds;
}

void FSFEUP02Model::reset() {
  state_ = std::make_shared<VehicleState>();
  execution_times_ = std::make_shared<VehicleModelExecutionTimes>();
  motor_->reset();
  battery_->reset();
  inverter_->reset();
}

std::string FSFEUP02Model::get_model_name() const { return "FSFEUP02Model"; }
