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

void FSFEUP03Model::update_ride_height() {
  const double front_mass_distribution =
      car_parameters_->cg_2_rear_axis / car_parameters_->wheelbase;
  const double static_front_load = car_parameters_->total_mass *
        car_parameters_->physical_constants->gravity * front_mass_distribution;
  const double static_rear_load = car_parameters_->total_mass *
        car_parameters_->physical_constants->gravity * (1.0 - front_mass_distribution);

  const double front_load_now = state_->wheels_vertical_load.front_left +
                                state_->wheels_vertical_load.front_right;
  const double rear_load_now = state_->wheels_vertical_load.rear_left +
                               state_->wheels_vertical_load.rear_right;

  // First-order approximation: treats the entire axle load delta, including the unsprung-mass
  // component, as spring deflection. We intentionally keep the larger simplification here while the
  // model remains a one-step lagged ride-height estimate.
  state_->ride_height_front = car_parameters_->aero_parameters->ride_height_front -
      (front_load_now - static_front_load) /
          (2.0 * car_parameters_->load_transfer_parameters->front_wheel_rate);
  state_->ride_height_rear = car_parameters_->aero_parameters->ride_height_rear -
      (rear_load_now - static_rear_load) /
          (2.0 * car_parameters_->load_transfer_parameters->rear_wheel_rate);
}

void FSFEUP03Model::step(double dt, common_lib::structures::Wheels throttle, double angle) {
  using Clock = std::chrono::steady_clock;

    // Motor + battery
  const auto powertrain_start = Clock::now();
  auto [torque_left, current_left]   = calculate_side_powertrain(
    throttle.rear_left, state_->wheels_speed.rear_left, motor_left_, drive_left_, true);
  auto [torque_right, current_right] = calculate_side_powertrain(
    throttle.rear_right, state_->wheels_speed.rear_right, motor_right_, drive_right_, false);
 
    double total_requested = current_left + current_right;
    double total_allowed = battery_->calculate_allowed_current(total_requested);
    // If the total requested current is greater than the total allowed current, both currents are scaled down proportionally to their requested values
    double scale = (total_requested > 1e-6) ? std::min(1.0, total_allowed / total_requested) : 1.0;
 
    state_->motor_current_left  = current_left  * scale;
    state_->motor_current_right = current_right * scale;
    state_->motor_torque_left   = torque_left   * scale;
    state_->motor_torque_right  = torque_right  * scale;
 
    // Advance internal motor/battery state (thermal, SOC) with the arbitrated values
    motor_left_->update_state(state_->motor_current_left, state_->motor_torque_left, dt);
    motor_right_->update_state(state_->motor_current_right, state_->motor_torque_right, dt);
    const double total_battery_current = state_->motor_current_left + state_->motor_current_right;
    battery_->update_state(total_battery_current, dt);


    state_->motor_thermal_state = motor_left_->get_thermal_state();
    state_->motor_thermal_capacity = motor_left_->get_thermal_capacity();
    state_->battery_current = battery_->get_current();
    state_->battery_voltage = battery_->get_voltage();
    state_->battery_soc = battery_->get_soc();
    state_->battery_open_circuit_voltage = battery_->get_open_circuit_voltage();
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
  update_ride_height();
  aero_->update_ride_height(state_->ride_height_front, state_->ride_height_rear);
  const auto aero_start = Clock::now();
  update_ride_height();
  aero_->update_ride_height(state_->ride_height_front, state_->ride_height_rear);
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
  // Calculate brake torques from brake input
  common_lib::structures::Wheels brake_torques = {0.0, 0.0, 0.0, 0.0};
  const double brake_input = -std::min(throttle.rear_left, throttle.rear_right);
  if (brake_input > 0.0) {
    brake_torques = brake_->calculate_brake_torques(brake_input);
  }

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

void FSFEUP03Model::update_ride_height() {
  const double aero_load_front =
      std::abs(state_->aero_downforce) * car_parameters_->aero_parameters->aero_balance_front;
  const double aero_load_rear =
      std::abs(state_->aero_downforce) * (1.0 - car_parameters_->aero_parameters->aero_balance_front);

  // Only the spring-reacted component -- excludes unsprung inertia and anti-squat/anti-dive geometry.
  const double elastic_longitudinal_transfer =
      load_transfer_->calculate_elastic_longitudinal_transfer(state_->ax);

  // Lateral/roll transfer deliberately excluded -- cancels exactly in the axle average by symmetry.
  state_->ride_height_front = car_parameters_->aero_parameters->ride_height_front -
      (aero_load_front - elastic_longitudinal_transfer / 2.0) /
          (2.0 * car_parameters_->load_transfer_parameters->front_wheel_rate);
  state_->ride_height_rear = car_parameters_->aero_parameters->ride_height_rear -
      (aero_load_rear + elastic_longitudinal_transfer / 2.0) /
          (2.0 * car_parameters_->load_transfer_parameters->rear_wheel_rate);
}

void FSFEUP03Model::reset() {
  state_ = std::make_shared<VehicleState>();
  execution_times_ = std::make_shared<VehicleModelExecutionTimes>();

  const double front_mass_distribution =
      car_parameters_->cg_2_rear_axis / car_parameters_->wheelbase;
  const double static_front_load = car_parameters_->total_mass *
        car_parameters_->physical_constants->gravity * front_mass_distribution;
  const double static_rear_load = car_parameters_->total_mass *
        car_parameters_->physical_constants->gravity * (1.0 - front_mass_distribution);

  state_->wheels_vertical_load = {static_front_load / 2.0, static_front_load / 2.0,
                                 static_rear_load / 2.0, static_rear_load / 2.0};
  state_->ride_height_front = car_parameters_->aero_parameters->ride_height_front;
  state_->ride_height_rear = car_parameters_->aero_parameters->ride_height_rear;

  motor_left_->reset();
  motor_right_->reset();
  battery_->reset();
  inverter_->reset();
}

std::string FSFEUP03Model::get_model_name() const { return "FSFEUP03Model"; }

std::pair<double, double> FSFEUP03Model::calculate_side_powertrain(
    double throttle_input, double wheel_speed,
    const std::shared_ptr<MotorModel>& motor,
    const std::shared_ptr<IndependentDriveModel>& drive,
    bool left_side) {
    const double motor_omega = drive->calculate_motor_omega(wheel_speed);
    if (left_side) {
      state_->motor_omega_left = motor_omega;
    } else {
      state_->motor_omega_right = motor_omega;
    }
    state_->motor_omega = 0.5 * (state_->motor_omega_left + state_->motor_omega_right);

    const double motor_rpm = std::abs(motor_omega * 60.0 / (2.0 * M_PI));

    const double max_torque = motor->get_max_torque_at_rpm(motor_rpm);
    const double reference_torque = throttle_input * max_torque;
    const double efficiency = motor->get_efficiency(std::abs(reference_torque), motor_rpm);
    const double requested_current =
      reference_torque / (car_parameters_->motor_parameters->kt_constant * efficiency);

  return {reference_torque, requested_current};
}