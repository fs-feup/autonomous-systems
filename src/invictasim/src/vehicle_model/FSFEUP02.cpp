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
  double throttle_input =
      (throttle.rear_left + throttle.rear_right) / 2.0;  // Average throttle for rear-wheel drive
  double motor_torque = calculate_powertrain_torque(throttle_input, dt);
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

double FSFEUP02Model::get_battery_current() const { return battery_->getCurrent(); }

double FSFEUP02Model::get_battery_voltage() const { return battery_->getVoltage(); }

double FSFEUP02Model::get_battery_soc() const { return battery_->getSoC(); }

double FSFEUP02Model::calculate_powertrain_torque(double throttle_input, double dt) {
  // Calculate motor RPM from wheel speed and gear ratio
  float avg_wheel_speed = (state_->wheels_speed.rear_left + state_->wheels_speed.rear_right) / 2.0f;
  float wheel_angular_velocity = avg_wheel_speed /
                                 simulator_parameters_->car_parameters.wheel_diameter * 2.0f *
                                 M_PI;  // rad/s
  float motor_omega = wheel_angular_velocity * simulator_parameters_->car_parameters.gear_ratio;
  float motor_rpm = (motor_omega * 60.0f / (2.0f * M_PI));

  // Get maximum torque available at this RPM
  float max_motor_torque = motor_->getMaxTorqueAtRPM(motor_rpm);

  // Step 1: Torque Request and Mechanical Power
  float requested_motor_torque = throttle_input * max_motor_torque;
  float mechanical_power_req = requested_motor_torque * motor_omega;

  // Step 2: Electrical Power Requirement
  float efficiency = motor_->getEfficiency(requested_motor_torque, motor_rpm);
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
      battery_->calculateCurrentForPower(electrical_power_req);

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
  battery_->updateState(actual_current, dt);
  motor_->updateState(actual_current, actual_motor_torque, dt);

  return actual_motor_torque;
}
