#include "vehicle_model/FSFEUP03.hpp"

#include <algorithm>
#include <cmath>

FSFEUP03Model::FSFEUP03Model(const InvictaSimParameters& simulator_parameters)
    : VehicleModel(simulator_parameters) {
  this->tire_model_ = tire_models_map.at(simulator_parameters.tire_model.c_str())(
      simulator_parameters.car_parameters);
  this->motor_left_ = motor_models_map.at(simulator_parameters.motor_model.c_str())(
    simulator_parameters.car_parameters);
  this->motor_right_ = motor_models_map.at(simulator_parameters.motor_model.c_str())(
    simulator_parameters.car_parameters);
  this->battery_ = battery_models_map.at(simulator_parameters.battery_model.c_str())(
    simulator_parameters.car_parameters);
  this->drive_left_ =
      independent_drive_models_map.at(simulator_parameters.transmission_model.c_str())(
          simulator_parameters.car_parameters);
  this->drive_right_ =
      independent_drive_models_map.at(simulator_parameters.transmission_model.c_str())(
          simulator_parameters.car_parameters);
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

void FSFEUP03Model::step(double dt, common_lib::structures::Wheels throttle, double angle) {
  using Clock = std::chrono::steady_clock;

  // Motor + battery
  const auto powertrain_start = Clock::now();
  auto [torque_left, current_left]   = calculate_side_powertrain(
    throttle.rear_left, state_->wheels_speed.rear_left, motor_left_, drive_left_);
  auto [torque_right, current_right] = calculate_side_powertrain(
    throttle.rear_right, state_->wheels_speed.rear_right, motor_right_, drive_right_);

    double total_requested = current_left + current_right;
    double total_allowed = battery_->calculate_allowed_current(total_requested);
    // To be confirmed: If the total requested current is greater than the total allowed current, both currents are scaled down proportionally to their requested values
    double scale = (total_requested > 1e-6) ? std::min(1.0, total_allowed / total_requested) : 1.0;

    state_->motor_current_left  = current_left  * scale;
    state_->motor_current_right = current_right * scale;
    state_->motor_torque_left   = torque_left   * scale;
    state_->motor_torque_right  = torque_right  * scale;
  const auto powertrain_end = Clock::now();

  // Apply per-side drive losses
    const auto transmission_start = Clock::now();
    state_->wheels_torque.front_left = 0.0;
    state_->wheels_torque.front_right = 0.0;
    state_->wheels_torque.rear_left = drive_left_->calculate_wheel_torque(
        state_->motor_torque_left, state_->wheels_speed.rear_left);
    state_->wheels_torque.rear_right = drive_right_->calculate_wheel_torque(
        state_->motor_torque_right, state_->wheels_speed.rear_right);
    const auto transmission_end = Clock::now();

  // Aerodynamics
  // based on implementation, this forces are negative by default, so we add them
  const auto aero_start = Clock::now();
  const Eigen::Vector3d aero_forces =
      aero_->aero_forces(Eigen::Vector3d(state_->vx, state_->vy, state_->yaw_rate));
  state_->aero_drag = aero_forces[0];
  state_->aero_downforce = aero_forces[2];
  const auto aero_end = Clock::now();

  // Steering
  const auto steering_start = Clock::now();
  const double steering_rate =
      this->steering_motor_->compute_steering_rate(state_->steering_angle, angle);
  state_->steering_angle += steering_rate * dt;

  auto steering = this->steering_->calculate_steering_angles(state_->steering_angle);
  double actual_steering_fl = steering[0];
  double actual_steering_fr = steering[1];
  const auto steering_end = Clock::now();

  // Load Transfer
  const auto load_transfer_start = Clock::now();
  state_->wheels_vertical_load = load_transfer_->compute_loads(
      LoadTransferInput{state_->ax, state_->ay, aero_forces[2]});  // aero_forces{Fx,Fy,Fz}
  const auto load_transfer_end = Clock::now();

  // Tire
  const auto tire_start = Clock::now();
  TireInput tire_input;
  tire_input.dt = dt;
  tire_input.vx = state_->vx;
  tire_input.vy = state_->vy;
  tire_input.yaw_rate = state_->yaw_rate;

  tire_input.tire = FL;
  tire_input.steering_angle = actual_steering_fl;
  tire_input.wheel_angular_speed = state_->wheels_speed.front_left;
  tire_input.vertical_load = state_->wheels_vertical_load.front_left;
  tire_input.last_slip_ratio =
      Eigen::Vector4d(state_->wheels_slip_ratio.front_left, state_->wheels_slip_ratio.front_right,
                      state_->wheels_slip_ratio.rear_left, state_->wheels_slip_ratio.rear_right);
  tire_input.last_slip_angle =
      Eigen::Vector4d(state_->wheels_slip_angle.front_left, state_->wheels_slip_angle.front_right,
                      state_->wheels_slip_angle.rear_left, state_->wheels_slip_angle.rear_right);
  state_->front_left_forces = this->tire_model_->calculate_tire_forces(tire_input);
  state_->wheels_slip_ratio.front_left = tire_input.slip_ratio;
  state_->wheels_slip_angle.front_left = tire_input.slip_angle;

  tire_input.tire = FR;
  tire_input.steering_angle = actual_steering_fr;
  tire_input.wheel_angular_speed = state_->wheels_speed.front_right;
  tire_input.vertical_load = state_->wheels_vertical_load.front_right;
  tire_input.last_slip_ratio =
      Eigen::Vector4d(state_->wheels_slip_ratio.front_left, state_->wheels_slip_ratio.front_right,
                      state_->wheels_slip_ratio.rear_left, state_->wheels_slip_ratio.rear_right);
  tire_input.last_slip_angle =
      Eigen::Vector4d(state_->wheels_slip_angle.front_left, state_->wheels_slip_angle.front_right,
                      state_->wheels_slip_angle.rear_left, state_->wheels_slip_angle.rear_right);
  state_->front_right_forces = this->tire_model_->calculate_tire_forces(tire_input);
  state_->wheels_slip_ratio.front_right = tire_input.slip_ratio;
  state_->wheels_slip_angle.front_right = tire_input.slip_angle;

  tire_input.tire = RL;
  tire_input.steering_angle = 0.0;  // Rear wheels do not steer
  tire_input.wheel_angular_speed = state_->wheels_speed.rear_left;
  tire_input.vertical_load = state_->wheels_vertical_load.rear_left;
  tire_input.last_slip_ratio =
      Eigen::Vector4d(state_->wheels_slip_ratio.front_left, state_->wheels_slip_ratio.front_right,
                      state_->wheels_slip_ratio.rear_left, state_->wheels_slip_ratio.rear_right);
  tire_input.last_slip_angle =
      Eigen::Vector4d(state_->wheels_slip_angle.front_left, state_->wheels_slip_angle.front_right,
                      state_->wheels_slip_angle.rear_left, state_->wheels_slip_angle.rear_right);
  state_->rear_left_forces = this->tire_model_->calculate_tire_forces(tire_input);
  state_->wheels_slip_ratio.rear_left = tire_input.slip_ratio;
  state_->wheels_slip_angle.rear_left = tire_input.slip_angle;

  tire_input.tire = RR;
  tire_input.wheel_angular_speed = state_->wheels_speed.rear_right;
  tire_input.vertical_load = state_->wheels_vertical_load.rear_right;
  tire_input.last_slip_ratio =
      Eigen::Vector4d(state_->wheels_slip_ratio.front_left, state_->wheels_slip_ratio.front_right,
                      state_->wheels_slip_ratio.rear_left, state_->wheels_slip_ratio.rear_right);
  tire_input.last_slip_angle =
      Eigen::Vector4d(state_->wheels_slip_angle.front_left, state_->wheels_slip_angle.front_right,
                      state_->wheels_slip_angle.rear_left, state_->wheels_slip_angle.rear_right);
  state_->rear_right_forces = this->tire_model_->calculate_tire_forces(tire_input);
  state_->wheels_slip_ratio.rear_right = tire_input.slip_ratio;
  state_->wheels_slip_angle.rear_right = tire_input.slip_angle;
  const auto tire_end = Clock::now();

  const auto integration_start = Clock::now();
  // Update wheel speeds
  if (state_->ebs_active) {
    state_->wheels_speed = {0.0, 0.0, 0.0, 0.0};
  } else {
    double R = car_parameters_->tire_parameters->effective_tire_r;
    // Front: tire + rim inertia only; Rear: tire + rim + motor + transmission (reflected)
    double I_front = car_parameters_->front_wheel_inertia;
    double I_rear = car_parameters_->rear_wheel_inertia;
    auto update_wheel = [&](double &w_speed, double net_torque, double inertia, double brake_t) {
      const double brake_sign = 2.0 / M_PI * std::atan(10.0 * w_speed);
      double total_torque = net_torque - brake_t * brake_sign;
      double dw = (total_torque / inertia) * dt;
      double next_w = w_speed + dw;
      if (brake_t > 0.0 && w_speed * next_w <= 0.0) {
        w_speed = 0.0;
      } else {
        w_speed = next_w;
      }
    };

    // Rear Wheels: Input Torque (Contains transmission losses) - Tire Reaction - Rolling Resistance
    update_wheel(state_->wheels_speed.rear_left,
                 state_->wheels_torque.rear_left - (state_->rear_left_forces[0] * R) - state_->rear_left_forces[2],
                 I_rear, brake_torques.rear_left);
    update_wheel(state_->wheels_speed.rear_right,
                 state_->wheels_torque.rear_right - (state_->rear_right_forces[0] * R) - state_->rear_right_forces[2],
                 I_rear, brake_torques.rear_right);

    // Front Wheels: Tire Reaction - Bearing Drag - Rolling Resistance
    update_wheel(state_->wheels_speed.front_left,
                 -(state_->front_left_forces[0] * R) - (car_parameters_->front_bearing_drag * state_->wheels_speed.front_left) - state_->front_left_forces[2],
                 I_front, brake_torques.front_left);
    update_wheel(state_->wheels_speed.front_right,
                 -(state_->front_right_forces[0] * R) - (car_parameters_->front_bearing_drag * state_->wheels_speed.front_right) - state_->front_right_forces[2],
                 I_front, brake_torques.front_right);
  }

  // Vehicle State Update
  // Sum of all forces normalized to the vehicle coordinate system
  double Fx_fl = state_->front_left_forces[0] * cos(actual_steering_fl) -
                 state_->front_left_forces[1] * sin(actual_steering_fl);
  double Fy_fl = state_->front_left_forces[0] * sin(actual_steering_fl) +
                 state_->front_left_forces[1] * cos(actual_steering_fl);
  double Fx_fr = state_->front_right_forces[0] * cos(actual_steering_fr) -
                 state_->front_right_forces[1] * sin(actual_steering_fr);
  double Fy_fr = state_->front_right_forces[0] * sin(actual_steering_fr) +
                 state_->front_right_forces[1] * cos(actual_steering_fr);
  double total_fx =
      Fx_fl + Fx_fr + state_->rear_left_forces[0] + state_->rear_right_forces[0] + aero_forces[0];
  double total_fy =
      Fy_fl + Fy_fr + state_->rear_left_forces[1] + state_->rear_right_forces[1] + aero_forces[1];
  state_->total_force_x = total_fx;
  state_->total_force_y = total_fy;

  // Update accelerations
  state_->ax = total_fx / car_parameters_->total_mass + state_->vy * state_->yaw_rate;
  state_->ay = total_fy / car_parameters_->total_mass - state_->vx * state_->yaw_rate;

  // Update velocities
  state_->vx += state_->ax * dt;
  state_->vy += state_->ay * dt;

  // Prevent oscillations at very low speeds by forcing a dead stop
  double speed = std::sqrt(state_->vx * state_->vx + state_->vy * state_->vy);
  const double rear_throttle_input =
      std::max(std::abs(throttle.rear_left), std::abs(throttle.rear_right));
  if (speed < 0.05 && rear_throttle_input < 0.01) {
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

  double lr = car_parameters_->cg_2_rear_axis;  // Distance from CG to rear axle
  double lf = car_parameters_->wheelbase - lr;  // Distance from CG to front axle
  double half_width = car_parameters_->track_width / 2.0;

  // 1. Moment from Lateral Forces (Fy)
  double moment_fy =
      (Fy_fl + Fy_fr) * lf - (state_->rear_left_forces[1] + state_->rear_right_forces[1]) * lr;
  state_->moment_fy = moment_fy;

  // 2. Moment from Longitudinal Forces (Fx)
  double moment_fx =
      (Fx_fr - Fx_fl) * half_width                                                  // front axle
      + (state_->rear_right_forces[0] - state_->rear_left_forces[0]) * half_width;  // rear axle
  state_->moment_fx = moment_fx;

  // 3. Self-Aligning Moments (Mz) from the tires themselves
  double total_mz = state_->front_left_forces[3] + state_->front_right_forces[3] +
                    state_->rear_left_forces[3] + state_->rear_right_forces[3];
  state_->self_aligning_moment = total_mz;

  double total_torque = moment_fy + moment_fx + total_mz;
  state_->total_torque_z = total_torque;

  // Update yaw
  double yaw_a = total_torque / car_parameters_->Izz;
  state_->yaw_rate += yaw_a * dt;
  state_->yaw += state_->yaw_rate * dt;

  // Keep yaw within [-pi, pi]
  if (state_->yaw > M_PI) {
    state_->yaw -= 2.0 * M_PI;
  }
  if (state_->yaw < -M_PI) {
    state_->yaw += 2.0 * M_PI;
  }

  // Update X and Y positions
  // 1. Calculate Global Velocities
  double cos_yaw = cos(state_->yaw);
  double sin_yaw = sin(state_->yaw);

  double v_global_x = state_->vx * cos_yaw - state_->vy * sin_yaw;
  double v_global_y = state_->vx * sin_yaw + state_->vy * cos_yaw;

  // 2. Update Global Positions (Integration)
  state_->x += v_global_x * dt;
  state_->y += v_global_y * dt;
  const auto integration_end = Clock::now();

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
  execution_times_->integration_ms =
      std::chrono::duration<double, std::milli>(integration_end - integration_start).count();
}

void FSFEUP03Model::reset() {
  state_ = std::make_shared<VehicleState>();
  execution_times_ = std::make_shared<VehicleModelExecutionTimes>();
  motor_->reset();
  battery_->reset();
  inverter_->reset();
}

std::string FSFEUP03Model::get_model_name() const { return "FSFEUP03Model"; }

std::pair<double, double> FSFEUP03Model::calculate_side_powertrain(
    double throttle_input, double wheel_speed,
    const std::shared_ptr<MotorModel>& motor,
    const std::shared_ptr<IndependentDriveModel>& drive) {
    double motor_omega = drive->calculate_motor_omega(wheel_speed);
    double motor_rpm = std::abs(motor_omega * 60.0 / (2.0 * M_PI));

    double max_torque = motor->get_max_torque_at_rpm(motor_rpm);
    double reference_torque = throttle_input * max_torque;
    double efficiency = motor->get_efficiency(std::abs(reference_torque), motor_rpm);
    double requested_current =
      reference_torque / (car_parameters_->motor_parameters->kt_constant * efficiency);

  return {reference_torque, requested_current};
  
  // Motor Efficiency at this state
  double motor_efficiency = motor_->get_efficiency(std::abs(reference_motor_torque), motor_rpm);

  const double current_magnitude =
      std::abs(reference_motor_torque) /
      (car_parameters_->motor_parameters->kt_constant * std::max(motor_efficiency, 0.05));
  const double mechanical_power = reference_motor_torque * motor_omega;
  const double current_sign =
      std::abs(motor_omega) > 1e-3 ? std::copysign(1.0, mechanical_power)
                                   : std::copysign(1.0, reference_motor_torque);
  double requested_motor_current = current_sign * current_magnitude;

  // Calculate the allowed current from the battery
  double allowed_motor_current = battery_->calculate_allowed_current(requested_motor_current);

  // Actual motor torque limited by the battery
  double actual_motor_torque =
      allowed_motor_current * car_parameters_->motor_parameters->kt_constant * motor_efficiency;

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

  return static_cast<double>(actual_motor_torque);
}
