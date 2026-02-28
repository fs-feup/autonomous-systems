#include "vehicle_model/FSFEUP02.hpp"

FSFEUP02Model::FSFEUP02Model(const InvictaSimParameters& simulator_parameters)
    : VehicleModel(simulator_parameters) {
  this->front_left = tire_models_map.at(simulator_parameters.tire_model.c_str())(
      simulator_parameters.car_parameters);
  this->front_right = tire_models_map.at(simulator_parameters.tire_model.c_str())(
      simulator_parameters.car_parameters);
  this->rear_left = tire_models_map.at(simulator_parameters.tire_model.c_str())(
      simulator_parameters.car_parameters);
  this->rear_right = tire_models_map.at(simulator_parameters.tire_model.c_str())(
      simulator_parameters.car_parameters);
  this->motor_ = motor_models_map.at(simulator_parameters.motor_model.c_str())(
      simulator_parameters.car_parameters);
  this->battery_ = battery_models_map.at(simulator_parameters.battery_model.c_str())(
      simulator_parameters.car_parameters);
  this->differential_ = differential_models_map.at(simulator_parameters.differential_model.c_str())(
      simulator_parameters.car_parameters);
  this->aero_ = aero_models_map.at(simulator_parameters.aero_model.c_str())(
      simulator_parameters.car_parameters);
  this->load_transfer_ = load_transfer_models_map.at(
      simulator_parameters.load_transfer_model.c_str())(simulator_parameters.car_parameters);
}
void FSFEUP02Model::step(double dt, common_lib::structures::Wheels throttle, double angle) {
  // 1. Average throttle input for rear-wheel drive logic
  double throttle_input = (throttle.rear_left + throttle.rear_right) / 2.0;

  // 2. Calculate torque (this now uses state_->vx to determine motor RPM)
  double motor_torque = calculate_powertrain_torque(throttle_input, dt);

  double mass = simulator_parameters_->car_parameters.total_mass;
  double gear_ratio = simulator_parameters_->car_parameters.gear_ratio;
  double wheel_radius = simulator_parameters_->car_parameters.wheel_diameter / 2.0;

  double tractive_force = (motor_torque * gear_ratio) / wheel_radius;

  // Basic Resistance Force (Rolling Resistance + Aero Drag)
  // Crr is typically ~0.015 for race tires [cite: 16]
  double rolling_resistance = 0.5 * mass * 9.81;
  double drag_force = 0.5 * 1.225 * 1.2 * 1.0 * std::pow(state_->vx, 2);

  // Net force determines acceleration
  double net_force = tractive_force;
  if (state_->vx > 0.1) {
    net_force -= (rolling_resistance + drag_force);
  } else if (std::abs(tractive_force) < rolling_resistance) {
    // Static friction: if torque is too low, don't move
    net_force = 0.0;
  }

  double acceleration = net_force / mass;

  // 4. Linear Integration of Velocity (v = v0 + a*dt)
  state_->vx += acceleration * dt;
  state_->vx = std::min(state_->vx, 33.3);  // Prevent negative velocity (no reverse logic)

  // Safety clamp: prevent negative velocity
  if (state_->vx < 0.0) state_->vx = 0.0;

  // 5. Update Wheel Speeds for the NEXT simulation step
  // motor_rpm in calculate_powertrain_torque relies on this
  double wheel_omega = state_->vx / wheel_radius;

  static int log_count2 = 0;
  if (log_count2 % 100 == 0) {
    RCLCPP_INFO(rclcpp::get_logger("FSFEUP02Model"),
                "Calculated Wheel Angular Velocity: %.2f rad/s", wheel_omega);
    RCLCPP_INFO(rclcpp::get_logger("FSFEUP02Model"), "Calculated Wheel Speed: %.2f m/s",
                state_->vx);
    log_count2 = 0;
  }
  log_count2++;
  state_->wheels_speed.rear_left = wheel_omega;
  state_->wheels_speed.rear_right = wheel_omega;
  state_->wheels_speed.front_left = wheel_omega;
  state_->wheels_speed.front_right = wheel_omega;

  // 6. Distribute torque to the wheels for state logging
  state_->wheels_torque =
      differential_->calculateTorqueDistribution(motor_torque, state_->wheels_speed);
}

void FSFEUP02Model::reset() {
  state_->x = 0.0;
  state_->y = 0.0;
  state_->z = 0.0;
  state_->roll = 0.0;
  state_->pitch = 0.0;
  state_->yaw = 0.0;
  state_->vx = 0.0;
  state_->vy = 0.0;
  state_->vz = 0.0;
}

std::string FSFEUP02Model::get_model_name() const { return "FSFEUP02Model"; }

double FSFEUP02Model::get_motor_torque() const { return motor_->get_torque(); }

double FSFEUP02Model::get_battery_current() const { return battery_->get_current(); }

double FSFEUP02Model::get_battery_voltage() const { return battery_->get_voltage(); }

double FSFEUP02Model::get_battery_soc() const { return battery_->get_soc(); }

/**
double FSFEUP02Model::calculate_powertrain_torque(double throttle_input, double dt) {
  // Calculate motor RPM from wheel speed and gear ratio
  float avg_wheel_speed = (state_->wheels_speed.rear_left + state_->wheels_speed.rear_right) / 2.0f;
  float wheel_angular_velocity = avg_wheel_speed /
                                 simulator_parameters_->car_parameters.wheel_diameter * 2.0f *
                                 M_PI;  // rad/s
  float motor_omega = wheel_angular_velocity * simulator_parameters_->car_parameters.gear_ratio;
  float motor_rpm = (motor_omega * 60.0f / (2.0f * M_PI));

  // Get maximum torque available at this RPM
  float max_motor_torque = motor_->get_max_torque_at_rpm(motor_rpm);

  // Step 1: Torque Request and Mechanical Power
  float requested_motor_torque = throttle_input * max_motor_torque;
  float mechanical_power_req = requested_motor_torque * motor_omega;

  // Step 2: Electrical Power Requirement
  float efficiency = motor_->get_efficiency(requested_motor_torque, motor_rpm);
  if (efficiency < 0.01f) {
    efficiency = 0.01f;  // Avoid division by zero
  }

  float electrical_power_req = 0.0f;
  if (mechanical_power_req >= 0.0f) {
    // Motoring
    electrical_power_req = mechanical_power_req / efficiency;
  } else {
    // Regenerative braking
    electrical_power_req = mechanical_power_req * efficiency;
  }

  // Step 3 & 4: Battery Current and Power Calculation with Limits
  // Battery model handles discriminant, all electrical limits (Imax, Vmin), and populates
  // both the feasible current and actual electrical power available
  float actual_current = 0.0f;
  float electrical_power_avail = 0.0f;
  std::tuple<float, float> battery_output =
      battery_->calculate_current_for_power(electrical_power_req);

  actual_current = std::get<0>(battery_output);
  electrical_power_avail = std::get<1>(battery_output);

  // Step 5: Available Torque Calculation
  float actual_motor_torque = 0.0f;
  float safe_omega = std::max(std::abs(motor_omega), 0.01f);
  if (electrical_power_avail >= 0.0f) {
    actual_motor_torque = (electrical_power_avail * efficiency) / safe_omega;
  } else {
    actual_motor_torque = (electrical_power_avail / efficiency) / safe_omega;
  }
  // Clamp to max torque
  actual_motor_torque = std::clamp(actual_motor_torque, -max_motor_torque, max_motor_torque);

  // Update battery and motor states
  battery_->update_state(actual_current, dt);
  motor_->update_state(actual_current, actual_motor_torque, dt);

  return actual_motor_torque;
}
*/

double FSFEUP02Model::calculate_powertrain_torque(double throttle_input, double dt) {
  // Calculate motor RPM from current wheel speed
  float avg_wheel_speed = (state_->wheels_speed.rear_left + state_->wheels_speed.rear_right) / 2.0f;
  float motor_omega = avg_wheel_speed * simulator_parameters_->car_parameters.gear_ratio;
  float motor_rpm = (motor_omega * 60.0f / (2.0f * M_PI));

  static int log_count = 0;
  if (log_count % 100 == 0) {
    RCLCPP_INFO(rclcpp::get_logger("FSFEUP02Model"), "Wheel Angular Velocity: %.2f rad/s",
                avg_wheel_speed);

    RCLCPP_INFO(rclcpp::get_logger("FSFEUP02Model"), "Motor RPM: %.2f", motor_rpm);
    log_count = 0;
  }
  log_count++;

  // Get maximum torque available at this RPM (Accounts for motor thermal state)
  float max_motor_torque = motor_->get_max_torque_at_rpm(motor_rpm);

  // Step 1: Torque Request and Mechanical Power
  float requested_motor_torque = (float)throttle_input * max_motor_torque;
  float mechanical_power_req = requested_motor_torque * motor_omega;

  // Step 2: Electrical Power Requirement
  float efficiency = motor_->get_efficiency(requested_motor_torque, motor_rpm);

  float electrical_power_req = 0.0f;
  if (mechanical_power_req >= 0.0f) {
    electrical_power_req = mechanical_power_req / efficiency;
  } else {
    electrical_power_req = mechanical_power_req * efficiency;
  }

  // FSG 80kW Limit: Ensure request doesn't exceed competition rules [cite: 1229]
  const float MAX_POWER_FSG = 80000.0f;
  electrical_power_req = std::clamp(electrical_power_req, -MAX_POWER_FSG, MAX_POWER_FSG);

  // Step 3 & 4: Battery Model solver
  std::tuple<float, float> battery_output =
      battery_->calculate_current_for_power(electrical_power_req);

  float actual_current = std::get<0>(battery_output);
  float electrical_power_avail = std::get<1>(battery_output);

  // Step 5: Available Torque Calculation
  // We use the ratio of power available to power requested to scale torque.
  // At standstill, power_req is 0, so we assume 100% torque availability if current is allowed.
  float power_scaling = 1.0f;
  if (std::abs(electrical_power_req) > 1.0f) {
    power_scaling = electrical_power_avail / electrical_power_req;
  }

  float actual_motor_torque = requested_motor_torque * power_scaling;

  // Final physical clamp
  actual_motor_torque = std::clamp(actual_motor_torque, -max_motor_torque, max_motor_torque);

  // Update battery (SoC and V_RC) and motor (thermal state) [cite: 1618, 1626]
  battery_->update_state(actual_current, dt);
  motor_->update_state(actual_current, actual_motor_torque, dt);

  return (double)actual_motor_torque;
}
