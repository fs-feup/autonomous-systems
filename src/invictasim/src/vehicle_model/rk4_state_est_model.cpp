#include "vehicle_model/rk4_state_est_model.hpp"

#include <algorithm>
#include <cmath>

RK4StateEstModel::RK4StateEstModel(const InvictaSimParameters& simulator_parameters)
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

  // Cache frequently accessed parameters.
  lr_ = car_parameters_->cg_2_rear_axis;
  lf_ = car_parameters_->wheelbase - lr_;
  half_width_ = car_parameters_->track_width * 0.5;
  wheel_radius_ = car_parameters_->tire_parameters->effective_tire_r;
  inertia_ = car_parameters_->tire_parameters->wheel_inertia;
  total_mass_ = car_parameters_->total_mass;
  Izz_ = car_parameters_->Izz;
  max_peak_torque_ = car_parameters_->motor_parameters->max_peak_torque;
}

RK4StateEstModel::StateVec RK4StateEstModel::get_state_derivative(const StateVec& s,
                                                                  double throttle_input,
                                                                  double steering_target, double dt,
                                                                  bool write_telemetry) {
  StateVec ds = StateVec::Zero();
  double motor_torque = throttle_input * max_peak_torque_;
  common_lib::structures::Wheels wheel_speeds_struct(s(FL_W), s(FR_W), s(RL_W), s(RR_W));
  common_lib::structures::Wheels torques_struct =
      transmission_->calculate_wheel_torques(motor_torque, wheel_speeds_struct);
  Eigen::Vector4d torques(torques_struct.front_left, torques_struct.front_right,
                          torques_struct.rear_left, torques_struct.rear_right);

  // Per-wheel steering angles.
  Eigen::Vector4d wheel_angles = steering_->calculate_steering_angles(s(ST_ANGLE));

  // Aerodynamic forces.
  Eigen::Vector3d aero_forces = aero_->aero_forces(Eigen::Vector3d(s(VX), s(VY), s(YAW_RATE)));

  // Load distribution
  LoadTransferInput load_transfer_input;
  load_transfer_input.longitudinal_acceleration = s(AX);
  load_transfer_input.lateral_acceleration = s(AY);
  load_transfer_input.downforce = aero_forces(2);
  common_lib::structures::Wheels load_distribution =
      load_transfer_->compute_loads(load_transfer_input);
  Eigen::Vector4d vertical_loads(load_distribution.front_left, load_distribution.front_right,
                                 load_distribution.rear_left, load_distribution.rear_right);

  // Tire forces
  Eigen::VectorXd tire_forces(16);
  Eigen::Vector4d slip_ratio = Eigen::Vector4d::Zero();
  Eigen::Vector4d slip_angle = Eigen::Vector4d::Zero();
  TireInput tire_input;
  tire_input.vx = s(VX);
  tire_input.vy = s(VY);
  tire_input.yaw_rate = s(YAW_RATE);
  tire_input.dt = 0.0;  // unused by the non-transient model
  tire_input.last_slip_angle = Eigen::Vector4d::Zero();
  tire_input.last_slip_ratio = Eigen::Vector4d::Zero();
  for (Tire tire : {FL, FR, RL, RR}) {
    tire_input.tire = tire;
    tire_input.steering_angle = wheel_angles(tire);
    tire_input.wheel_angular_speed = s(FL_W + tire);
    tire_input.vertical_load = vertical_loads(tire);
    tire_forces.segment<4>(tire * 4) =
        tire_model_->calculate_tire_forces_not_transient(tire_input);  // [Fx, Fy, My, Mz]
    slip_ratio(tire) = tire_input.last_slip_ratio(tire);
    slip_angle(tire) = tire_input.last_slip_angle(tire);
  }

  // Sum forces and moments in the vehicle frame.
  double total_fx = aero_forces(0);
  double total_fy = aero_forces(1);
  double total_torque = 0.0;
  double moment_fx = 0.0;
  double moment_fy = 0.0;
  double self_aligning_moment = 0.0;
  // Linearized lateral stiffness accumulators for the implicit vy/yaw damping below.
  constexpr double kLateralStiffness = 20.0;  // dFy/dalpha per unit Fz [1/rad] (estimate)
  double lat_Cy = 0.0;   // sum of dFy_veh/dalpha
  double lat_Crr = 0.0;  // sum weighted by arm_x^2 (yaw)
  for (Tire tire : {FL, FR, RL, RR}) {
    double fx_tire = tire_forces(tire * 4);
    double fy_tire = tire_forces(tire * 4 + 1);
    double mz_tire = tire_forces(tire * 4 + 3);

    double cos_delta = std::cos(wheel_angles(tire));
    double sin_delta = std::sin(wheel_angles(tire));

    double fx_veh = fx_tire * cos_delta - fy_tire * sin_delta;
    double fy_veh = fx_tire * sin_delta + fy_tire * cos_delta;

    total_fx += fx_veh;
    total_fy += fy_veh;

    double arm_x = (tire == FL || tire == FR) ? lf_ : -lr_;
    double arm_y = (tire == FL || tire == RL) ? half_width_ : -half_width_;

    double c_eff = kLateralStiffness * vertical_loads(tire) * cos_delta;
    lat_Cy += c_eff;
    lat_Crr += c_eff * arm_x * arm_x;

    moment_fy += arm_x * fy_veh;
    moment_fx += -arm_y * fx_veh;
    self_aligning_moment += mz_tire;
    total_torque += (arm_x * fy_veh) - (arm_y * fx_veh) + mz_tire;
  }

  // Damp at low speed to avoid numerical issues with the implicit integration.
  const double V_reg = std::max(std::abs(s(VX)), 1.0);
  double ax = total_fx / total_mass_ + s(VY) * s(YAW_RATE);
  double ay = total_fy / total_mass_ - s(VX) * s(YAW_RATE);
  ds(VX) = ax;
  ds(VY) = ay / (1.0 + dt * lat_Cy / (total_mass_ * V_reg));
  ds(AX) = ds(VX) - s(AX);
  ds(AY) = ds(VY) - s(AY);
  ds(YAW_RATE) = (total_torque / Izz_) / (1.0 + dt * lat_Crr / (Izz_ * V_reg));
  ds(YAW) = s(YAW_RATE);
  ds(PX) = s(VX) * std::cos(s(YAW)) - s(VY) * std::sin(s(YAW));
  ds(PY) = s(VX) * std::sin(s(YAW)) + s(VY) * std::cos(s(YAW));
  ds(ST_ANGLE) = steering_motor_->compute_steering_rate(s(ST_ANGLE), steering_target);

  // Wheel angular accelerations (linear blend to stabilize the stiff slip dynamics at low speed).
  constexpr double kLinearSlipStiffness = 34.6;
  constexpr double V_floor = 1.0;
  const double half_track = car_parameters_->track_width / 2.0;
  for (Tire tire : {FL, FR, RL, RR}) {
    double sign_r = 2.0 / M_PI * std::atan(10.0 * s(FL_W + tire));
    double net_torque = torques(tire) - tire_forces(tire * 4) * wheel_radius_ -
                        sign_r * std::abs(tire_forces(tire * 4 + 2));

    // Per-wheel longitudinal contact velocity
    bool is_rear = (tire == RL || tire == RR);
    double y_sign = (tire == FL || tire == RL) ? 1.0 : -1.0;
    double x_sign = is_rear ? -1.0 : 1.0;
    double l_axle = is_rear ? lr_ : lf_;
    double vx_hub = s(VX) - s(YAW_RATE) * y_sign * half_track;
    double vy_hub = s(VY) + s(YAW_RATE) * x_sign * l_axle;
    double Vcx = vx_hub * std::cos(wheel_angles(tire)) + vy_hub * std::sin(wheel_angles(tire));
    double Vw = s(FL_W + tire) * wheel_radius_;

    double denom = std::max({std::abs(Vw), std::abs(Vcx), V_floor});
    double kx = kLinearSlipStiffness * vertical_loads(tire);
    double wheel_damping = dt * kx * wheel_radius_ * wheel_radius_ / denom;
    ds(FL_W + tire) = net_torque / (inertia_ + wheel_damping);
  }

  // Publish telemetry only from the start-of-step (k1)
  if (write_telemetry) {
    state_->front_left_forces = tire_forces.segment<4>(FL * 4);
    state_->front_right_forces = tire_forces.segment<4>(FR * 4);
    state_->rear_left_forces = tire_forces.segment<4>(RL * 4);
    state_->rear_right_forces = tire_forces.segment<4>(RR * 4);
    state_->wheels_torque =
        common_lib::structures::Wheels(torques(0), torques(1), torques(2), torques(3));
    state_->wheels_vertical_load = common_lib::structures::Wheels(
        vertical_loads(0), vertical_loads(1), vertical_loads(2), vertical_loads(3));
    state_->wheels_slip_ratio =
        common_lib::structures::Wheels(slip_ratio(0), slip_ratio(1), slip_ratio(2), slip_ratio(3));
    state_->wheels_slip_angle =
        common_lib::structures::Wheels(slip_angle(0), slip_angle(1), slip_angle(2), slip_angle(3));
    state_->aero_drag = aero_forces(0);
    state_->aero_downforce = aero_forces(2);
    state_->total_force_x = total_fx;
    state_->total_force_y = total_fy;
    state_->moment_fx = moment_fx;
    state_->moment_fy = moment_fy;
    state_->self_aligning_moment = self_aligning_moment;
    state_->total_torque_z = total_torque;
  }

  return ds;
}

void RK4StateEstModel::step(double dt, common_lib::structures::Wheels throttle, double angle) {
  using Clock = std::chrono::steady_clock;
  const auto step_start = Clock::now();

  double throttle_input =
      (throttle.rear_left + throttle.rear_right) / 2.0;  // Average throttle for rear-wheel drive

  // Pack the current vehicle state into the integration vector.
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

  // RK4. The start-of-step evaluation (k1) writes the published telemetry.
  StateVec k1 = get_state_derivative(s, throttle_input, angle, dt, true);
  StateVec k2 = get_state_derivative(s + 0.5 * dt * k1, throttle_input, angle, dt, false);
  StateVec k3 = get_state_derivative(s + 0.5 * dt * k2, throttle_input, angle, dt, false);
  StateVec k4 = get_state_derivative(s + dt * k3, throttle_input, angle, dt, false);
  StateVec s_next = s + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);

  // Unpack the integrated state.
  state_->vx = s_next(VX);
  state_->vy = s_next(VY);
  state_->yaw_rate = s_next(YAW_RATE);
  state_->yaw = s_next(YAW);
  state_->x = s_next(PX);
  state_->y = s_next(PY);
  state_->steering_angle = s_next(ST_ANGLE);
  state_->wheels_speed = common_lib::structures::Wheels(s_next(FL_W), s_next(FR_W), s_next(RL_W),
                                                        s_next(RR_W));

  // Integrated acceleration states
  state_->ax = s_next(AX);
  state_->ay = s_next(AY);

  // Wrap heading
  if (state_->yaw > M_PI) state_->yaw -= 2.0 * M_PI;
  if (state_->yaw < -M_PI) state_->yaw += 2.0 * M_PI;

  // Prevent oscillations at very low speeds by forcing a dead stop
  double speed = std::sqrt(state_->vx * state_->vx + state_->vy * state_->vy);
  if (speed < 0.1 && std::abs(throttle_input) < 0.01) {
    state_->vx = 0.0;
    state_->vy = 0.0;
    state_->ax = 0.0;
    state_->ay = 0.0;
    state_->yaw_rate = 0.0;
    state_->wheels_speed = {0.0, 0.0, 0.0, 0.0};
    state_->wheels_slip_ratio = {0.0, 0.0, 0.0, 0.0};
    state_->wheels_slip_angle = {0.0, 0.0, 0.0, 0.0};
  }

  // Time keeping
  const auto step_end = Clock::now();
  execution_times_->tire_ms =
      std::chrono::duration<double, std::milli>(step_end - step_start).count();
}

void RK4StateEstModel::reset() {
  state_ = std::make_shared<VehicleState>();
  *execution_times_ = VehicleModelExecutionTimes{};
}

std::string RK4StateEstModel::get_model_name() const { return "rk4StateEstModel"; }
